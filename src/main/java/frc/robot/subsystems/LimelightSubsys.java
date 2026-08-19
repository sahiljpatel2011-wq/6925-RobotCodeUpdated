package frc.robot.subsystems;

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
import frc.robot.LimelightHelpers.RawFiducial;
import frc.robot.vision.HubAimMath;

public class LimelightSubsys extends SubsystemBase {
    public static final double kTargetHeightInches = HubAimMath.kHubTagHeightInches;
    public static final double kCameraForwardInches = -1.46;
    public static final double kCameraSideInches = 0.0;
    public static final double kCameraHeightInches = HubAimMath.kCameraHeightInches;
    public static final double kCameraMountAngleDegrees = HubAimMath.kCameraMountAngleDegrees;

    private static final int[] kBlueTagIDs = {17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32};
    private static final int[] kRedTagIDs  = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};

    private static final double kMinTagAreaPercent = 0.1;
    private static final double kMaxAmbiguity = 0.2;

    private final String name;
    private final Supplier<Pose2d> poseSupplier;
    private final DoubleSupplier headingSupplier;
    private final NetworkTable telemetryTable;
    private final StructPublisher<Pose2d> posePublisher;
    private final Field2d field = new Field2d();
    private boolean filterSet = false;
    private int requestedPipeline = 0;
    private double pipelineResumeTime = 0;

    public LimelightSubsys(String name, Supplier<Pose2d> poseSupplier, DoubleSupplier headingSupplier) {
        this.name = name;
        this.poseSupplier = poseSupplier;
        this.headingSupplier = headingSupplier;
        this.telemetryTable = NetworkTableInstance.getDefault().getTable("SmartDashboard/" + name);
        this.posePublisher = telemetryTable.getStructTopic("Estimated Robot Pose", Pose2d.struct).publish();

        LimelightHelpers.setCameraPose_RobotSpace(name,
            kCameraForwardInches * 0.0254,
            kCameraSideInches * 0.0254,
            kCameraHeightInches * 0.0254,
            0.0,
            kCameraMountAngleDegrees,
            0.0
        );

        // Shots were tuned with this POI. Do not zero until a range day.
        LimelightHelpers.setFiducial3DOffset(name, -0.5842, 0.0, 0.0);

        SmartDashboard.putData("Field", field);
    }

    @Override
    public void periodic() {
        LimelightHelpers.SetRobotOrientation(name, headingSupplier.getAsDouble(), 0, 0, 0, 0, 0);

        if (!filterSet) {
            final Optional<Alliance> alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
                if (alliance.get() == Alliance.Blue) {
                    LimelightHelpers.SetFiducialIDFiltersOverride(name, kBlueTagIDs);
                } else {
                    LimelightHelpers.SetFiducialIDFiltersOverride(name, kRedTagIDs);
                }
                filterSet = true;
            }
        }

        field.setRobotPose(poseSupplier.get());
        publishSimLimelight();
        SmartDashboard.putNumber("LL Pipeline", getPipelineIndex());
    }

    public void setPipeline(int index) {
        if (index == 0 && requestedPipeline != 0) {
            pipelineResumeTime = Timer.getFPGATimestamp() + 0.15;
        }
        requestedPipeline = index;
        LimelightHelpers.setPipelineIndex(name, index);
    }

    public double getPipelineIndex() {
        return LimelightHelpers.getCurrentPipelineIndex(name);
    }

    public boolean isAprilTagPipeline() {
        return requestedPipeline == 0 && Timer.getFPGATimestamp() >= pipelineResumeTime;
    }

    public Optional<RawFiducial> bestHubFiducial() {
        if (FeatureFlags.hubTagAimFilter() && !isAprilTagPipeline()) {
            return Optional.empty();
        }
        final boolean isBlue = Landmarks.isBlueAlliance();
        RawFiducial best = null;
        for (RawFiducial fiducial : LimelightHelpers.getRawFiducials(name)) {
            if (fiducial.ambiguity > kMaxAmbiguity || fiducial.ta < kMinTagAreaPercent) {
                continue;
            }
            if (FeatureFlags.hubTagAimFilter()) {
                if (Landmarks.isAllianceKnown()) {
                    if (!HubAimMath.isAllianceHubTag(fiducial.id, isBlue)) {
                        continue;
                    }
                } else if (!HubAimMath.isHubTag(fiducial.id)) {
                    continue;
                }
            } else if (!HubAimMath.isHubTag(fiducial.id)) {
                continue;
            }
            if (best == null || fiducial.ta > best.ta) {
                best = fiducial;
            }
        }
        if (best != null) {
            return Optional.of(best);
        }
        final int tid = (int) LimelightHelpers.getFiducialID(name);
        if (LimelightHelpers.getTV(name) && HubAimMath.isHubTag(tid)) {
            if (!FeatureFlags.hubTagAimFilter() || !Landmarks.isAllianceKnown()
                || HubAimMath.isAllianceHubTag(tid, isBlue)) {
                return Optional.of(new RawFiducial(
                    tid,
                    LimelightHelpers.getTX(name),
                    LimelightHelpers.getTY(name),
                    LimelightHelpers.getTA(name),
                    0, 0, 0
                ));
            }
        }
        return Optional.empty();
    }

    public Optional<Measurement> getMeasurement() {
        if (!FeatureFlags.visionPoseFuse() || !isAprilTagPipeline()) {
            return Optional.empty();
        }
        final PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
        return measurementFrom(poseEstimate, true);
    }

    public Optional<Measurement> getMegaTag1Measurement() {
        final PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
        return measurementFrom(poseEstimate, false);
    }

    private Optional<Measurement> measurementFrom(PoseEstimate poseEstimate, boolean xyOnly) {
        if (poseEstimate == null || poseEstimate.tagCount == 0 || poseEstimate.avgTagArea < kMinTagAreaPercent) {
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
        final Optional<Translation2d> hub = Landmarks.targetPositionOptional();
        final NetworkTable table = NetworkTableInstance.getDefault().getTable(name.startsWith("limelight") ? name : "limelight");
        if (hub.isEmpty()) {
            table.getEntry("tv").setNumber(0);
            table.getEntry("tid").setNumber(0);
            return;
        }
        final Translation2d toHub = hub.get().minus(pose.getTranslation());
        final double rangeInches = toHub.getNorm() / 0.0254;
        final double headingError = HubAimMath.poseAimTxDegrees(
            pose.getX(), pose.getY(), pose.getRotation().getRadians(),
            hub.get().getX(), hub.get().getY()
        );
        final double heightDiff = kTargetHeightInches - kCameraHeightInches;
        final double cameraToTag = Math.max(1.0, rangeInches - HubAimMath.kHubCenterOffsetInches);
        final double ty = Math.toDegrees(Math.atan(heightDiff / cameraToTag)) - kCameraMountAngleDegrees;
        final int tid = Landmarks.isBlueAlliance() ? 18 : 2;
        table.getEntry("tv").setNumber(1);
        table.getEntry("tx").setNumber(headingError);
        table.getEntry("ty").setNumber(ty);
        table.getEntry("ta").setNumber(1.0);
        table.getEntry("tid").setNumber(tid);
        table.getEntry("cl").setNumber(0);
        table.getEntry("tl").setNumber(0);
        field.getObject("Hub").setPose(new Pose2d(hub.get(), Rotation2d.kZero));
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
