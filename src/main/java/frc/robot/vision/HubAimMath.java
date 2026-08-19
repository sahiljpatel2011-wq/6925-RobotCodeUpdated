package frc.robot.vision;

import java.util.OptionalDouble;

/**
 * Pure hub / trench aim math for 6925. Numbers match the live robot
 * (26° camera, 8 in lateral, 23.5 in hub depth). No HAL.
 */
public final class HubAimMath {
    public static final double kCameraHeightInches = 25.39;
    public static final double kCameraMountAngleDegrees = 26.0;
    public static final double kHubTagHeightInches = 44.25;
    public static final double kHubCenterOffsetInches = 23.5;
    public static final double kLateralOffsetInches = 8.0;
    public static final double kPassAimOffsetDegrees = 15.0;
    public static final double kMinTanAbs = 0.02;

    public static final int[] kRedHubTags = {2, 3, 4, 5, 8, 9, 10, 11};
    public static final int[] kBlueHubTags = {18, 19, 20, 21, 24, 25, 26, 27};
    public static final int[] kTrenchTags = {1, 6, 7, 12, 17, 22, 23, 28};

    private HubAimMath() {}

    public static boolean isHubTag(int id) {
        return contains(kRedHubTags, id) || contains(kBlueHubTags, id);
    }

    public static boolean isAllianceHubTag(int id, boolean isBlue) {
        return contains(isBlue ? kBlueHubTags : kRedHubTags, id);
    }

    public static boolean isTrenchTag(int id) {
        return contains(kTrenchTags, id);
    }

    /** Offset-right tags: hub center is left of the tag (−8 in). */
    public static double lateralOffsetInches(int tagId) {
        return switch (tagId) {
            case 3, 8, 19, 24 -> -kLateralOffsetInches;
            case 9, 11, 25, 27 -> kLateralOffsetInches;
            default -> 0.0;
        };
    }

    /**
     * Ground-plane camera-to-tag range from ty. Empty if tan is near 0°/90°.
     */
    public static OptionalDouble cameraToTagInches(double tyDegrees) {
        final double angleDeg = kCameraMountAngleDegrees + tyDegrees;
        if (!Double.isFinite(angleDeg)) {
            return OptionalDouble.empty();
        }
        final double wrapped = Math.abs(((angleDeg % 180.0) + 180.0) % 180.0);
        if (wrapped < 2.0 || Math.abs(wrapped - 90.0) < 2.0) {
            return OptionalDouble.empty();
        }
        final double tan = Math.tan(Math.toRadians(angleDeg));
        if (!Double.isFinite(tan) || Math.abs(tan) < kMinTanAbs) {
            return OptionalDouble.empty();
        }
        final double range = (kHubTagHeightInches - kCameraHeightInches) / tan;
        if (!Double.isFinite(range) || range <= 0.0 || range > 400.0) {
            return OptionalDouble.empty();
        }
        return OptionalDouble.of(range);
    }

    public static OptionalDouble hubRangeInches(int tagId, double txDegrees, double tyDegrees) {
        if (!isHubTag(tagId)) {
            return OptionalDouble.empty();
        }
        final OptionalDouble cameraToTag = cameraToTagInches(tyDegrees);
        if (cameraToTag.isEmpty()) {
            return OptionalDouble.empty();
        }
        return OptionalDouble.of(cameraToTag.getAsDouble() + kHubCenterOffsetInches);
    }

    /**
     * Corrected tx (degrees) that aims at hub center rather than the tag face.
     */
    public static OptionalDouble hubAimTxDegrees(int tagId, double txDegrees, double tyDegrees) {
        if (!isHubTag(tagId)) {
            return OptionalDouble.empty();
        }
        final OptionalDouble cameraToTag = cameraToTagInches(tyDegrees);
        if (cameraToTag.isEmpty()) {
            return OptionalDouble.empty();
        }
        final double rawTxRad = Math.toRadians(txDegrees);
        final double cam = cameraToTag.getAsDouble();
        final double hubLateral = cam * Math.sin(rawTxRad) + lateralOffsetInches(tagId);
        final double hubForward = cam * Math.cos(rawTxRad) + kHubCenterOffsetInches;
        if (Math.abs(hubForward) < 1e-6) {
            return OptionalDouble.empty();
        }
        return OptionalDouble.of(Math.toDegrees(Math.atan2(hubLateral, hubForward)));
    }

    /**
     * Heading error (degrees) from robot pose to a hub XY, robot-relative "tx"
     * (positive means hub is to the right, matching Limelight tx).
     */
    public static double poseAimTxDegrees(
        double robotXMeters,
        double robotYMeters,
        double robotHeadingRadians,
        double hubXMeters,
        double hubYMeters
    ) {
        final double dx = hubXMeters - robotXMeters;
        final double dy = hubYMeters - robotYMeters;
        final double desired = Math.atan2(dy, dx);
        double error = desired - robotHeadingRadians;
        while (error > Math.PI) {
            error -= 2.0 * Math.PI;
        }
        while (error < -Math.PI) {
            error += 2.0 * Math.PI;
        }
        return Math.toDegrees(error);
    }

    public static double poseRangeInches(
        double robotXMeters,
        double robotYMeters,
        double hubXMeters,
        double hubYMeters
    ) {
        final double dx = hubXMeters - robotXMeters;
        final double dy = hubYMeters - robotYMeters;
        return Math.hypot(dx, dy) / 0.0254;
    }

    /**
     * Inward yaw offset for a trench tag so a pass aims at field center.
     * Same 15° convention as the original 7/12/23/28 mapping.
     */
    public static double passAimOffsetDegrees(int tagId) {
        return switch (tagId) {
            case 6, 12, 22, 28 -> -kPassAimOffsetDegrees;
            case 1, 7, 17, 23 -> kPassAimOffsetDegrees;
            default -> 0.0;
        };
    }

    private static boolean contains(int[] ids, int id) {
        for (int candidate : ids) {
            if (candidate == id) {
                return true;
            }
        }
        return false;
    }
}
