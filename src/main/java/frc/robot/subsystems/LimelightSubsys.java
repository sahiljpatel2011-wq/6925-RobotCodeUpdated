package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FeatureFlags;
import frc.robot.Landmarks;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.LimelightHelpers.RawDetection;
import frc.robot.LimelightHelpers.RawFiducial;
import frc.robot.vision.FuelDetect;
import frc.robot.vision.HubAimMath;
import frc.robot.vision.VisionGates;

public class LimelightSubsys extends SubsystemBase {
    public static final double kTargetHeightInches = HubAimMath.kHubTagHeightInches;
    public static final double kCameraForwardInches = -1.46;
    public static final double kCameraSideInches = 0.0;
    public static final double kCameraHeightInches = HubAimMath.kCameraHeightInches;
    public static final double kCameraMountAngleDegrees = HubAimMath.kCameraMountAngleDegrees;

    private static final int[] kBlueTagIDs = {17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32};
    private static final int[] kRedTagIDs  = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};

    private static final double kMinTagAreaPercent = VisionGates.kMinTagAreaPercent;

    private final String name;
    private final Supplier<Pose2d> poseSupplier;
    private final DoubleSupplier headingSupplier;
    private final DoubleSupplier yawRateSupplier;
    private final NetworkTable telemetryTable;
    private final StructPublisher<Pose2d> posePublisher;
    private final Field2d field = new Field2d();
    private boolean filterSet = false;
    private Alliance lastFilterAlliance = null;
    private int requestedPipeline = 0;
    private double pipelineResumeTime = 0;
    private double pipelineReadyTime = 0;
    private double lastHeartbeat = -1;
    private boolean flushOrientation = true;

    public LimelightSubsys(String name, Supplier<Pose2d> poseSupplier, DoubleSupplier headingSupplier) {
        this(name, poseSupplier, headingSupplier, () -> 0.0);
    }

    public LimelightSubsys(
        String name,
        Supplier<Pose2d> poseSupplier,
        DoubleSupplier headingSupplier,
        DoubleSupplier yawRateSupplier
    ) {
        this.name = name;
        this.poseSupplier = poseSupplier;
        this.headingSupplier = headingSupplier;
        this.yawRateSupplier = yawRateSupplier;
        this.telemetryTable = NetworkTableInstance.getDefault().getTable("SmartDashboard/" + name);
        this.posePublisher = telemetryTable.getStructTopic("Estimated Robot Pose", Pose2d.struct).publish();

        applyStaticConfig();
        SmartDashboard.putData("Field", field);
    }

    @Override
    public void periodic() {
        maybeReapplyConfig();
        double yaw = 0.0;
        double yawRate = 0.0;
        try {
            yaw = headingSupplier.getAsDouble();
            yawRate = yawRateSupplier.getAsDouble();
        } catch (RuntimeException ignored) {
        }
        if (flushOrientation) {
            LimelightHelpers.SetRobotOrientation(name, yaw, yawRate, 0, 0, 0, 0);
            flushOrientation = false;
        } else {
            LimelightHelpers.SetRobotOrientation_NoFlush(name, yaw, yawRate, 0, 0, 0, 0);
        }

        if (!filterSet || allianceChanged()) {
            final Optional<Alliance> alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
                if (alliance.get() == Alliance.Blue) {
                    LimelightHelpers.SetFiducialIDFiltersOverride(name, kBlueTagIDs);
                } else {
                    LimelightHelpers.SetFiducialIDFiltersOverride(name, kRedTagIDs);
                }
                lastFilterAlliance = alliance.get();
                filterSet = true;
            }
        }

        field.setRobotPose(poseSupplier.get());
        publishSimLimelight();
        publishDetectionHud();
        SmartDashboard.putNumber("LL Pipeline", getPipelineIndex());
    }

    private boolean allianceChanged() {
        final Optional<Alliance> alliance = DriverStation.getAlliance();
        return lastFilterAlliance != null && alliance.isPresent() && alliance.get() != lastFilterAlliance;
    }

    public void setPipeline(int index) {
        if (index != requestedPipeline) {
            pipelineReadyTime = Timer.getFPGATimestamp() + 0.15;
            if (index == 0) {
                pipelineResumeTime = pipelineReadyTime;
            }
        }
        requestedPipeline = index;
        applyPipelineCrop();
        LimelightHelpers.setPipelineIndex(name, index);
    }

    /** Tags are not in the hopper; do not crop pipeline 0 or close-range hub tags disappear. */
    private void applyPipelineCrop() {
        if (requestedPipeline == 1) {
            LimelightHelpers.setCropWindow(name, -1.0, 1.0, -1.0, 0.55);
        } else {
            LimelightHelpers.setCropWindow(name, -1.0, 1.0, -1.0, 1.0);
        }
    }

    private void applyStaticConfig() {
        LimelightHelpers.setCameraPose_RobotSpace(name,
            kCameraForwardInches * 0.0254,
            kCameraSideInches * 0.0254,
            kCameraHeightInches * 0.0254,
            0.0,
            kCameraMountAngleDegrees,
            0.0
        );
        LimelightHelpers.setFiducial3DOffset(name, -0.5842, 0.0, 0.0);
        LimelightHelpers.SetIMUMode(name, 0);
        LimelightHelpers.setStreamMode_Standard(name);
        LimelightHelpers.SetThrottle(name, 0);
        applyPipelineCrop();
        LimelightHelpers.setPipelineIndex(name, requestedPipeline);
    }

    private void maybeReapplyConfig() {
        final double hb = LimelightHelpers.getHeartbeat(name);
        final boolean reconnect = hb > 0 && (lastHeartbeat < 0 || hb + 20 < lastHeartbeat);
        if (reconnect) {
            applyStaticConfig();
            filterSet = false;
            flushOrientation = true;
        }
        if (hb > 0) {
            lastHeartbeat = hb;
        }
    }

    public double getPipelineIndex() {
        return LimelightHelpers.getCurrentPipelineIndex(name);
    }

    public boolean isAprilTagPipeline() {
        return requestedPipeline == 0 && Timer.getFPGATimestamp() >= pipelineResumeTime;
    }

    public boolean isFuelPipelineReady() {
        return requestedPipeline == 1 && Timer.getFPGATimestamp() >= pipelineReadyTime;
    }

    public Optional<RawFiducial> bestHubFiducial() {
        RawFiducial best = null;
        for (RawFiducial fiducial : hubFiducials()) {
            if (best == null || fiducial.ta > best.ta) {
                best = fiducial;
            }
        }
        return Optional.ofNullable(best);
    }

    /**
     * Area-weighted hub tx/range so a closer tag is trusted more than a far one.
     */
    public Optional<HubAimMath.WeightedHubAim> weightedHubAim() {
        final RawFiducial[] tags = hubFiducials();
        if (tags.length == 0) {
            return Optional.empty();
        }
        final int[] ids = new int[tags.length];
        final double[] tx = new double[tags.length];
        final double[] ty = new double[tags.length];
        final double[] ta = new double[tags.length];
        for (int i = 0; i < tags.length; i++) {
            ids[i] = tags[i].id;
            tx[i] = tags[i].txnc;
            ty[i] = tags[i].tync;
            ta[i] = tags[i].ta;
        }
        return HubAimMath.areaWeightedHubAim(ids, tx, ty, ta);
    }

    public Optional<RawDetection> bestFuelDetection() {
        if (!isFuelPipelineReady()) {
            return Optional.empty();
        }
        RawDetection best = null;
        for (RawDetection detection : LimelightHelpers.getRawDetections(name)) {
            if (!FuelDetect.isFieldFuel(detection.classId, detection.ta, detection.tync)) {
                continue;
            }
            if (best == null || detection.ta > best.ta) {
                best = detection;
            }
        }
        return Optional.ofNullable(best);
    }

    private RawFiducial[] hubFiducials() {
        if (FeatureFlags.hubTagAimFilter() && !isAprilTagPipeline()) {
            return new RawFiducial[0];
        }
        final boolean isBlue = Landmarks.isBlueAlliance();
        final RawFiducial[] raw = LimelightHelpers.getRawFiducials(name);
        final ArrayList<RawFiducial> valid = new ArrayList<>(raw.length);
        for (RawFiducial fiducial : raw) {
            if (fiducial.ta < kMinTagAreaPercent) {
                continue;
            }
            if (!isUsableHubTag(fiducial.id, isBlue)) {
                continue;
            }
            valid.add(fiducial);
        }
        if (!valid.isEmpty()) {
            return valid.toArray(new RawFiducial[0]);
        }
        final int tid = (int) LimelightHelpers.getFiducialID(name);
        final double ta = LimelightHelpers.getTA(name);
        if (LimelightHelpers.getTV(name) && ta >= kMinTagAreaPercent && isUsableHubTag(tid, isBlue)) {
            return new RawFiducial[] {
                new RawFiducial(
                    tid,
                    LimelightHelpers.getTX(name),
                    LimelightHelpers.getTY(name),
                    ta,
                    0, 0, 0
                )
            };
        }
        return new RawFiducial[0];
    }

    private boolean isUsableHubTag(int id, boolean isBlue) {
        if (FeatureFlags.hubTagAimFilter()) {
            if (Landmarks.isAllianceKnown()) {
                return HubAimMath.isAllianceHubTag(id, isBlue);
            }
            return HubAimMath.isHubTag(id);
        }
        return HubAimMath.isHubTag(id);
    }

    public Optional<Measurement> getMeasurement() {
        if (!FeatureFlags.visionPoseFuse() || !isAprilTagPipeline()) {
            return Optional.empty();
        }
        final PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
        return measurementFrom(poseEstimate, true, false);
    }

    public Optional<Measurement> getMegaTag1Measurement() {
        if (!isAprilTagPipeline()) {
            return Optional.empty();
        }
        final PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
        return measurementFrom(poseEstimate, false, true);
    }

    private Optional<Measurement> measurementFrom(PoseEstimate poseEstimate, boolean xyOnly, boolean seed) {
        if (poseEstimate == null) {
            return Optional.empty();
        }
        final double x = poseEstimate.pose.getX();
        final double y = poseEstimate.pose.getY();
        final boolean valid = seed
            ? VisionGates.isValidMegaTag1Seed(poseEstimate.tagCount, poseEstimate.avgTagArea, poseEstimate.avgTagDist, x, y)
            : VisionGates.isValidMegaTag2Fuse(poseEstimate.tagCount, poseEstimate.avgTagArea, poseEstimate.avgTagDist, x, y);
        if (!valid) {
            return Optional.empty();
        }

        final double distance = poseEstimate.avgTagDist;
        final double xyStdDev = poseEstimate.tagCount >= 2
            ? 0.02 * distance
            : 0.05 * distance * distance;
        final double thetaStdDev = xyOnly ? 9999.0 : 0.5;
        final Matrix<N3, N1> standardDeviations = VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev);

        posePublisher.set(poseEstimate.pose);
        field.getObject("Vision").setPose(poseEstimate.pose);
        return Optional.of(new Measurement(poseEstimate, standardDeviations));
    }

    private void publishSimLimelight() {
        if (!RobotBase.isSimulation()) {
            return;
        }
        final Pose2d pose = poseSupplier.get();
        final Translation2d hub = Landmarks.targetPositionOptional().orElse(Landmarks.kBlueHub);
        final NetworkTable table = NetworkTableInstance.getDefault().getTable(
            name.startsWith("limelight") ? name : "limelight");
        final double heading = pose.getRotation().getRadians();
        final double camForwardM = kCameraForwardInches * 0.0254;
        final double camX = pose.getX() + Math.cos(heading) * camForwardM;
        final double camY = pose.getY() + Math.sin(heading) * camForwardM;
        final double[] tagXy = HubAimMath.tagXyMeters(camX, camY, hub.getX(), hub.getY());
        final double cameraToTagInches = Math.hypot(tagXy[0] - camX, tagXy[1] - camY) / 0.0254;
        if (cameraToTagInches < 8.0) {
            table.getEntry("tv").setNumber(0);
            table.getEntry("tid").setNumber(0);
            table.getEntry("botpose_wpiblue").setDoubleArray(new double[0]);
            table.getEntry("botpose_orb_wpiblue").setDoubleArray(new double[0]);
            return;
        }
        final double tagTx = HubAimMath.poseAimTxDegrees(
            camX, camY, heading, tagXy[0], tagXy[1]);
        final double ty = HubAimMath.tyDegreesFromCameraToTagInches(cameraToTagInches);
        final int tid = HubAimMath.preferredHubTagId(Landmarks.isBlueAlliance());
        table.getEntry("tv").setNumber(1);
        table.getEntry("tx").setNumber(tagTx);
        table.getEntry("ty").setNumber(ty);
        table.getEntry("ta").setNumber(1.0);
        table.getEntry("tid").setNumber(tid);
        table.getEntry("cl").setNumber(0);
        table.getEntry("tl").setNumber(0);
        table.getEntry("rawfiducials").setDoubleArray(new double[] {
            tid, tagTx, ty, 1.0, cameraToTagInches * 0.0254, cameraToTagInches * 0.0254, 0.05
        });
        final double[] botpose = new double[] {
            pose.getX(), pose.getY(), 0.0,
            0.0, 0.0, pose.getRotation().getDegrees(),
            0.0, 1.0, 0.5, cameraToTagInches * 0.0254, 1.0
        };
        table.getEntry("botpose_wpiblue").setDoubleArray(botpose);
        table.getEntry("botpose_orb_wpiblue").setDoubleArray(botpose);
        field.getObject("Hub").setPose(new Pose2d(hub, Rotation2d.kZero));
    }

    private void publishDetectionHud() {
        int tagCount = 0;
        int bestHubId = 0;
        int bestAnyId = 0;
        double bestHubTa = -1.0;
        double bestAnyTa = -1.0;
        for (RawFiducial fiducial : LimelightHelpers.getRawFiducials(name)) {
            if (fiducial.ta < kMinTagAreaPercent) {
                continue;
            }
            tagCount++;
            if (fiducial.ta > bestAnyTa) {
                bestAnyTa = fiducial.ta;
                bestAnyId = fiducial.id;
            }
            if (HubAimMath.isHubTag(fiducial.id) && fiducial.ta > bestHubTa) {
                bestHubTa = fiducial.ta;
                bestHubId = fiducial.id;
            }
        }
        SmartDashboard.putNumber("LL Tag Count", tagCount);
        SmartDashboard.putNumber("LL Best Tag", bestHubId != 0 ? bestHubId : bestAnyId);

        int fuelCount = 0;
        double fuelTx = 0.0;
        double bestFuelTa = -1.0;
        if (isFuelPipelineReady()) {
            for (RawDetection detection : LimelightHelpers.getRawDetections(name)) {
                if (!FuelDetect.isFieldFuel(detection.classId, detection.ta, detection.tync)) {
                    continue;
                }
                fuelCount++;
                if (detection.ta > bestFuelTa) {
                    bestFuelTa = detection.ta;
                    fuelTx = detection.txnc;
                }
            }
        }
        SmartDashboard.putNumber("LL Fuel Count", fuelCount);
        SmartDashboard.putNumber("LL Fuel tx", fuelTx);
    }

    public static class Measurement {
        public final PoseEstimate poseEstimate;
        public final Matrix<N3, N1> standardDeviations;

        public Measurement(PoseEstimate poseEstimate, Matrix<N3, N1> standardDeviations) {
            this.poseEstimate = poseEstimate;
            this.standardDeviations = standardDeviations;
        }
    }
}
