package frc.robot.vision;

/**
 * Floor-FUEL filters for 6925's Limelight pipeline 1 (official Fuel B1).
 * Rejects hopper/ceiling blobs so onboard balls are not treated as field Fuel.
 */
public final class FuelDetect {
    public static final double kMinTaPercent = 0.15;
    public static final double kMaxTaPercent = 8.0;
    public static final double kMaxTyDegrees = 8.0;
    public static final double kMinTyDegrees = -22.0;

    private FuelDetect() {}

    /**
     * Fuel B1 is a single-class detector (class 0). Ignore other classes.
     * Positive ty is the top of the image (ceiling / hub). Huge ta is onboard hopper.
     */
    public static boolean isFieldFuel(int classId, double taPercent, double tyDegrees) {
        if (classId != 0) {
            return false;
        }
        if (!Double.isFinite(taPercent) || taPercent < kMinTaPercent || taPercent > kMaxTaPercent) {
            return false;
        }
        if (!Double.isFinite(tyDegrees) || tyDegrees > kMaxTyDegrees || tyDegrees < kMinTyDegrees) {
            return false;
        }
        return true;
    }
}
