package frc.robot.vision;

import java.util.Optional;
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
    /** Same "On Target" window already on the HUD. */
    public static final double kOnTargetDegrees = 2.0;
    /** Same 0.15 s window the HUD already uses for a fresh tx. */
    public static final double kAimHoldSeconds = 0.15;

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

    public static boolean isUsableHubTag(int id, boolean allianceKnown, boolean isBlue) {
        if (allianceKnown) {
            return isAllianceHubTag(id, isBlue);
        }
        return isHubTag(id);
    }

    /**
     * Largest-area hub tag in a raw-fiducial list. Trench / opponent tags are ignored.
     *
     * @return index into the arrays, or -1
     */
    public static int bestHubIndex(
        int[] ids,
        double[] taPercent,
        boolean allianceKnown,
        boolean isBlue
    ) {
        if (ids == null || taPercent == null) {
            return -1;
        }
        final int n = Math.min(ids.length, taPercent.length);
        int best = -1;
        double bestTa = -1.0;
        for (int i = 0; i < n; i++) {
            if (!Double.isFinite(taPercent[i]) || taPercent[i] < VisionGates.kMinTagAreaPercent) {
                continue;
            }
            if (!isUsableHubTag(ids[i], allianceKnown, isBlue)) {
                continue;
            }
            if (taPercent[i] > bestTa) {
                bestTa = taPercent[i];
                best = i;
            }
        }
        return best;
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
     * Area-weighted hub aim across several valid tags. Larger ta (closer / more
     * pixels) pulls tx and range more than a distant tag.
     */
    public static Optional<WeightedHubAim> areaWeightedHubAim(
        int[] ids,
        double[] txDegrees,
        double[] tyDegrees,
        double[] taPercent
    ) {
        if (ids == null || txDegrees == null || tyDegrees == null || taPercent == null) {
            return Optional.empty();
        }
        final int n = Math.min(Math.min(ids.length, txDegrees.length), Math.min(tyDegrees.length, taPercent.length));
        double weightTx = 0.0;
        double weightRange = 0.0;
        double txWeight = 0.0;
        double rangeWeight = 0.0;
        int bestId = 0;
        double bestTa = -1.0;
        int count = 0;
        for (int i = 0; i < n; i++) {
            if (!isHubTag(ids[i]) || !Double.isFinite(taPercent[i]) || taPercent[i] <= 0.0) {
                continue;
            }
            final OptionalDouble aimTx = hubAimTxDegrees(ids[i], txDegrees[i], tyDegrees[i]);
            final OptionalDouble range = hubRangeInches(ids[i], txDegrees[i], tyDegrees[i]);
            final double usedTx = aimTx.isPresent() ? aimTx.getAsDouble() : txDegrees[i];
            if (!Double.isFinite(usedTx)) {
                continue;
            }
            weightTx += usedTx * taPercent[i];
            txWeight += taPercent[i];
            if (range.isPresent()) {
                weightRange += range.getAsDouble() * taPercent[i];
                rangeWeight += taPercent[i];
            }
            count++;
            if (taPercent[i] > bestTa) {
                bestTa = taPercent[i];
                bestId = ids[i];
            }
        }
        if (count == 0 || txWeight <= 0.0) {
            return Optional.empty();
        }
        final double avgRange = rangeWeight > 0.0 ? weightRange / rangeWeight : Double.NaN;
        return Optional.of(new WeightedHubAim(weightTx / txWeight, avgRange, bestId, count));
    }

    /** Center hub tag on the alliance-station face (what 6925 sees when shooting). */
    public static int preferredHubTagId(boolean isBlue) {
        return isBlue ? 26 : 10;
    }

    public static final class WeightedHubAim {
        public final double txDegrees;
        public final double rangeInches;
        public final int bestId;
        public final int count;

        public WeightedHubAim(double txDegrees, double rangeInches, int bestId, int count) {
            this.txDegrees = txDegrees;
            this.rangeInches = rangeInches;
            this.bestId = bestId;
            this.count = count;
        }
    }

    /**
     * Limelight-style tx (degrees) from robot pose to a hub XY.
     * Positive means the hub is to the right of the nose (same as Limelight tx).
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
        final double cos = Math.cos(robotHeadingRadians);
        final double sin = Math.sin(robotHeadingRadians);
        // WPILib robot frame: +X forward, +Y left. Limelight +tx is right.
        final double forward = dx * cos + dy * sin;
        final double left = -dx * sin + dy * cos;
        return Math.toDegrees(Math.atan2(-left, forward));
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

    /** Limelight ty that a hub tag at this ground range would report. */
    public static double tyDegreesFromCameraToTagInches(double cameraToTagInches) {
        if (cameraToTagInches <= 0.0) {
            return 0.0;
        }
        return Math.toDegrees(Math.atan((kHubTagHeightInches - kCameraHeightInches) / cameraToTagInches))
            - kCameraMountAngleDegrees;
    }

    /**
     * Hub-tag XY: 23.5 in from hub center toward the robot (tag face the camera sees).
     */
    public static double[] tagXyMeters(
        double robotXMeters,
        double robotYMeters,
        double hubXMeters,
        double hubYMeters
    ) {
        final double dx = robotXMeters - hubXMeters;
        final double dy = robotYMeters - hubYMeters;
        final double dist = Math.hypot(dx, dy);
        if (dist < 1e-6) {
            return new double[] {hubXMeters, hubYMeters};
        }
        final double offsetM = kHubCenterOffsetInches * 0.0254;
        return new double[] {
            hubXMeters + dx / dist * offsetM,
            hubYMeters + dy / dist * offsetM
        };
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

    /** Original kP aim so the robot keeps turning onto the hub. */
    public static double aimAssistOmega(double txDegrees, double kP) {
        if (!Double.isFinite(txDegrees) || !Double.isFinite(kP)) {
            return 0.0;
        }
        return -txDegrees * kP;
    }

    public static boolean keepLastAim(double nowSeconds, double lastSeenSeconds, double holdSeconds) {
        if (!Double.isFinite(nowSeconds) || !Double.isFinite(lastSeenSeconds) || !Double.isFinite(holdSeconds)) {
            return false;
        }
        if (lastSeenSeconds < 0.0 || holdSeconds <= 0.0) {
            return false;
        }
        return nowSeconds - lastSeenSeconds <= holdSeconds;
    }

    public static boolean isOnTarget(double txDegrees) {
        return Double.isFinite(txDegrees) && Math.abs(txDegrees) < kOnTargetDegrees;
    }

    /**
     * Start feeding only when spun up and aimed. Once feeding, keep feeding
     * so a 2° flicker cannot cut the volley.
     */
    public static boolean autoFeed(boolean atSpeed, boolean onTarget, boolean alreadyFeeding) {
        return alreadyFeeding || (atSpeed && onTarget);
    }

    /**
     * Limelight hub range minus distance closed during look-ahead. Uses existing
     * kLookAheadSeconds so moving shots use the same table as standing shots.
     */
    public static double movingShotInches(
        double cameraHubInches,
        double robotXMeters,
        double robotYMeters,
        double hubXMeters,
        double hubYMeters,
        double fieldVxMetersPerSec,
        double fieldVyMetersPerSec,
        double lookAheadSeconds
    ) {
        if (!Double.isFinite(cameraHubInches) || cameraHubInches <= 0.0) {
            return 75.125;
        }
        if (!(lookAheadSeconds > 0.0)
            || !Double.isFinite(fieldVxMetersPerSec)
            || !Double.isFinite(fieldVyMetersPerSec)) {
            return cameraHubInches;
        }
        final double dx = hubXMeters - robotXMeters;
        final double dy = hubYMeters - robotYMeters;
        final double dist = Math.hypot(dx, dy);
        if (dist < 1e-3) {
            return cameraHubInches;
        }
        final double closingMeters =
            (fieldVxMetersPerSec * dx + fieldVyMetersPerSec * dy) / dist * lookAheadSeconds;
        return cameraHubInches - closingMeters / 0.0254;
    }

    /**
     * Extra Limelight-style tx (deg) so the nose leads the hub while translating.
     * Standing still is 0.
     */
    public static double movingLeadTxDegrees(
        double robotXMeters,
        double robotYMeters,
        double headingRadians,
        double hubXMeters,
        double hubYMeters,
        double fieldVxMetersPerSec,
        double fieldVyMetersPerSec,
        double lookAheadSeconds
    ) {
        if (!(lookAheadSeconds > 0.0)
            || !Double.isFinite(fieldVxMetersPerSec)
            || !Double.isFinite(fieldVyMetersPerSec)) {
            return 0.0;
        }
        final double now = poseAimTxDegrees(
            robotXMeters, robotYMeters, headingRadians, hubXMeters, hubYMeters);
        final double future = poseAimTxDegrees(
            robotXMeters + fieldVxMetersPerSec * lookAheadSeconds,
            robotYMeters + fieldVyMetersPerSec * lookAheadSeconds,
            headingRadians,
            hubXMeters,
            hubYMeters);
        double lead = future - now;
        if (!Double.isFinite(lead)) {
            return 0.0;
        }
        while (lead > 180.0) {
            lead -= 360.0;
        }
        while (lead < -180.0) {
            lead += 360.0;
        }
        return lead;
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
