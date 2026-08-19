package frc.robot.vision;

/**
 * Accept/reject rules for Limelight MegaTag poses. Kept HAL-free so tests can
 * prove seed vs fuse without a camera.
 */
public final class VisionGates {
    public static final double kMinTagAreaPercent = 0.1;
    public static final double kMaxFuseTagDistMeters = 7.0;
    public static final double kMaxSingleTagFuseDistMeters = 4.0;
    public static final double kMaxSeedTagDistMeters = 8.0;
    public static final double kInvalidOriginMeters = 0.10;
    public static final double kUnseededPoseMeters = 0.50;

    private VisionGates() {}

    /**
     * MegaTag2 XY fuse during teleop/auto. Drops (0,0), far 1-tag, and long-range
     * estimates that yank odometry.
     */
    public static boolean isValidMegaTag2Fuse(
        int tagCount,
        double avgTagArea,
        double avgTagDist,
        double poseXMeters,
        double poseYMeters
    ) {
        if (!isValidCommon(tagCount, avgTagArea, avgTagDist, poseXMeters, poseYMeters, kMaxFuseTagDistMeters)) {
            return false;
        }
        return !(tagCount == 1 && avgTagDist > kMaxSingleTagFuseDistMeters);
    }

    /**
     * MegaTag1 disabled seed. One tag on the wall is enough; do not use the
     * 4 m 1-tag fuse gate or the robot never leaves (0,0).
     */
    public static boolean isValidMegaTag1Seed(
        int tagCount,
        double avgTagArea,
        double avgTagDist,
        double poseXMeters,
        double poseYMeters
    ) {
        return isValidCommon(tagCount, avgTagArea, avgTagDist, poseXMeters, poseYMeters, kMaxSeedTagDistMeters);
    }

    /**
     * First good measurement from an uninitialized pose is always a large jump.
     * Rejecting it would trap odometry at the origin for the whole match.
     */
    public static boolean allowVisionJump(
        double currentXMeters,
        double currentYMeters,
        double jumpMeters,
        double maxJumpMeters
    ) {
        if (!Double.isFinite(jumpMeters) || jumpMeters < 0.0) {
            return false;
        }
        if (Math.hypot(currentXMeters, currentYMeters) < kUnseededPoseMeters) {
            return true;
        }
        return jumpMeters <= maxJumpMeters;
    }

    private static boolean isValidCommon(
        int tagCount,
        double avgTagArea,
        double avgTagDist,
        double poseXMeters,
        double poseYMeters,
        double maxDistMeters
    ) {
        if (tagCount < 1) {
            return false;
        }
        if (!Double.isFinite(avgTagArea) || avgTagArea < kMinTagAreaPercent) {
            return false;
        }
        if (!Double.isFinite(avgTagDist) || avgTagDist <= 0.0 || avgTagDist > maxDistMeters) {
            return false;
        }
        if (!Double.isFinite(poseXMeters) || !Double.isFinite(poseYMeters)) {
            return false;
        }
        return Math.hypot(poseXMeters, poseYMeters) >= kInvalidOriginMeters;
    }
}
