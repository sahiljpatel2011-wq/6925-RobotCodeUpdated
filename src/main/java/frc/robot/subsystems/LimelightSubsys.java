package frc.robot.subsystems;

import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FeatureFlags;
import frc.robot.Landmarks;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.LimelightHelpers.RawFiducial;
import frc.robot.vision.HubAimMath;
import frc.robot.vision.VisionGates;

/**
 * AprilTag Limelight only. Does not rewrite pipeline/crop/IMU/tag-filters —
 * those live on the camera. Software picks a hub tag if the camera's
 * primary target is a trench tag.
 */
public class LimelightSubsys extends SubsystemBase {
    public static final double kTargetHeightInches = HubAimMath.kHubTagHeightInches;
    public static final double kCameraForwardInches = -1.46;
    public static final double kCameraSideInches = 0.0;
    public static final double kCameraHeightInches = HubAimMath.kCameraHeightInches;
    public static final double kCameraMountAngleDegrees = HubAimMath.kCameraMountAngleDegrees;

    private final String name;
    private final Supplier<Pose2d> poseSupplier;
    private final DoubleSupplier yawRateSupplier;
    private final NetworkTable telemetryTable;
    private final StructPublisher<Pose2d> posePublisher;
    private double lastHeartbeat = -1;
    private double lastHeartbeatChangeTime = -1;
    private boolean flushOrientation = true;

    public LimelightSubsys(String name, Supplier<Pose2d> poseSupplier) {
        this(name, poseSupplier, () -> 0.0);
    }

    public LimelightSubsys(String name, Supplier<Pose2d> poseSupplier, DoubleSupplier yawRateSupplier) {
        this.name = name;
        this.poseSupplier = poseSupplier;
        this.yawRateSupplier = yawRateSupplier;
        this.telemetryTable = NetworkTableInstance.getDefault().getTable("SmartDashboard/" + name);
        this.posePublisher = telemetryTable.getStructTopic("Estimated Robot Pose", Pose2d.struct).publish();
        applyCameraGeometry();
    }

    private void applyCameraGeometry() {
        LimelightHelpers.setCameraPose_RobotSpace(name,
            kCameraForwardInches * 0.0254,
            kCameraSideInches * 0.0254,
            kCameraHeightInches * 0.0254,
            0.0,
            kCameraMountAngleDegrees,
            0.0
        );
        LimelightHelpers.setFiducial3DOffset(name, -0.5842, 0.0, 0.0);
        LimelightHelpers.setPipelineIndex(name, 0);
    }

    @Override
    public void periodic() {
        try {
            maybeReapplyGeometry();
            double yaw = 0.0;
            double yawRate = 0.0;
            try {
                yaw = poseSupplier.get().getRotation().getDegrees();
                yawRate = yawRateSupplier.getAsDouble();
            } catch (RuntimeException ignored) {
            }
            if (flushOrientation) {
                LimelightHelpers.SetRobotOrientation(name, yaw, yawRate, 0, 0, 0, 0);
                flushOrientation = false;
            } else {
                LimelightHelpers.SetRobotOrientation_NoFlush(name, yaw, yawRate, 0, 0, 0, 0);
            }
            publishTagHud();
        } catch (RuntimeException ignored) {
        }
    }

    private void publishTagHud() {
        int tagCount = 0;
        int bestHubId = 0;
        int bestAnyId = 0;
        double bestHubTa = -1.0;
        double bestAnyTa = -1.0;
        for (RawFiducial fiducial : LimelightHelpers.getRawFiducials(name)) {
            if (fiducial.ta < VisionGates.kMinTagAreaPercent) {
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
        SmartDashboard.putBoolean("LL TV", LimelightHelpers.getTV(name));
        SmartDashboard.putNumber("LL Heartbeat", LimelightHelpers.getHeartbeat(name));
        SmartDashboard.putNumber("LL Pipeline", getPipelineIndex());
    }

    /** If the Limelight reboots after the Rio, re-send mount pose only — not pipeline/crop. */
    private void maybeReapplyGeometry() {
        final double hb = LimelightHelpers.getHeartbeat(name);
        final boolean reconnect = hb > 0 && (lastHeartbeat < 0 || hb + 20 < lastHeartbeat);
        if (reconnect) {
            applyCameraGeometry();
            flushOrientation = true;
        }
        if (hb > 0) {
            if (hb != lastHeartbeat) {
                lastHeartbeatChangeTime = Timer.getFPGATimestamp();
            }
            lastHeartbeat = hb;
        }
    }

    public double getPipelineIndex() {
        return LimelightHelpers.getCurrentPipelineIndex(name);
    }

    /**
     * True when the Limelight is publishing. Heartbeat can stall on NT glitches,
     * so a visible tag also counts as alive.
     */
    public boolean isStreaming() {
        if (lastHeartbeatChangeTime > 0
            && Timer.getFPGATimestamp() - lastHeartbeatChangeTime < 2.0) {
            return true;
        }
        try {
            return LimelightHelpers.getTV(name) || LimelightHelpers.getRawFiducials(name).length > 0;
        } catch (RuntimeException e) {
            return false;
        }
    }

    public boolean isAprilTagPipeline() {
        return Math.abs(getPipelineIndex()) < 0.5;
    }

    /**
     * Hub tag for RB aim. Uses the camera's primary tx/ty when that tag is a hub
     * tag (original feel). If the camera locked a trench tag, uses the largest
     * hub tag still in the frame.
     */
    public Optional<HubTagAim> hubAimTag() {
        final boolean allianceKnown = Landmarks.isAllianceKnown();
        final boolean isBlue = Landmarks.isBlueAlliance();
        final int tid = (int) LimelightHelpers.getFiducialID(name);
        if (LimelightHelpers.getTV(name) && HubAimMath.isUsableHubTag(tid, allianceKnown, isBlue)) {
            return Optional.of(new HubTagAim(
                tid,
                LimelightHelpers.getTX(name),
                LimelightHelpers.getTY(name)
            ));
        }
        final RawFiducial[] raw = LimelightHelpers.getRawFiducials(name);
        final int n = raw.length;
        final int[] ids = new int[n];
        final double[] ta = new double[n];
        for (int i = 0; i < n; i++) {
            ids[i] = raw[i].id;
            ta[i] = raw[i].ta;
        }
        final int index = HubAimMath.bestHubIndex(ids, ta, allianceKnown, isBlue);
        if (index < 0) {
            return Optional.empty();
        }
        return Optional.of(new HubTagAim(raw[index].id, raw[index].txnc, raw[index].tync));
    }

    public Optional<Measurement> getMeasurement() {
        if (!FeatureFlags.visionPoseFuse()) {
            return Optional.empty();
        }
        final PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
        return measurementFrom(poseEstimate, false);
    }

    public Optional<Measurement> getMegaTag1Measurement() {
        final PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
        return measurementFrom(poseEstimate, true);
    }

    private Optional<Measurement> measurementFrom(PoseEstimate poseEstimate, boolean seed) {
        if (poseEstimate == null) {
            return Optional.empty();
        }
        final double x = poseEstimate.pose.getX();
        final double y = poseEstimate.pose.getY();
        final boolean valid = seed
            ? VisionGates.isValidMegaTag1Seed(
                poseEstimate.tagCount, poseEstimate.avgTagArea, poseEstimate.avgTagDist, x, y)
            : VisionGates.isValidMegaTag2Fuse(
                poseEstimate.tagCount, poseEstimate.avgTagArea, poseEstimate.avgTagDist, x, y);
        if (!valid) {
            return Optional.empty();
        }

        final double distance = poseEstimate.avgTagDist;
        final double xyStdDev = poseEstimate.tagCount >= 2
            ? 0.02 * distance
            : 0.05 * distance * distance;
        // Gyro owns heading. Vision only corrects XY so MegaTag cannot yank yaw.
        final double thetaStdDev = 9999.0;
        final Matrix<N3, N1> standardDeviations = VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev);

        posePublisher.set(poseEstimate.pose);
        return Optional.of(new Measurement(poseEstimate, standardDeviations));
    }

    public static final class HubTagAim {
        public final int id;
        public final double txDegrees;
        public final double tyDegrees;

        public HubTagAim(int id, double txDegrees, double tyDegrees) {
            this.id = id;
            this.txDegrees = txDegrees;
            this.tyDegrees = tyDegrees;
        }
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
