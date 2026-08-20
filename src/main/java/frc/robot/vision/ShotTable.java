package frc.robot.vision;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static frc.robot.Constants.ShooterConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Single source for 6925's live shot table (Constants + 150 RPM).
 * Always uses the compiled table so Dashboard edits cannot change a match.
 */
public final class ShotTable {
    public static final class Shot {
        public final double shooterRPM;
        public final double hoodPosition;

        public Shot(double shooterRPM, double hoodPosition) {
            this.shooterRPM = shooterRPM;
            this.hoodPosition = hoodPosition;
        }
    }

    public static final double kLiveRpmOffset = 150.0;

    private static final InterpolatingTreeMap<Distance, Shot> kCompiledTable =
        new InterpolatingTreeMap<>(
            (startValue, endValue, q) ->
                InverseInterpolator.forDouble()
                    .inverseInterpolate(startValue.in(Meters), endValue.in(Meters), q.in(Meters)),
            (startValue, endValue, t) ->
                new Shot(
                    Interpolator.forDouble().interpolate(startValue.shooterRPM, endValue.shooterRPM, t),
                    Interpolator.forDouble().interpolate(startValue.hoodPosition, endValue.hoodPosition, t)
                )
        );

    static {
        kCompiledTable.put(Inches.of(47.0), liveShot(kFixedShotRPM, kHoodAt47in));
        kCompiledTable.put(Inches.of(50.0), liveShot(kRPMAt50in, kHoodAt50in));
        kCompiledTable.put(Inches.of(75.125), liveShot(kRPMAt75in, kHoodAt75in));
        kCompiledTable.put(Inches.of(84.0), liveShot(kFixedShotRPM, kHoodAt84in));
        kCompiledTable.put(Inches.of(92.0), liveShot(kRPMAt92in, kHoodAt92in));
        kCompiledTable.put(Inches.of(100.0), liveShot(kRPMAt100in, kHoodAt100in));
        kCompiledTable.put(Inches.of(110.0), liveShot(kRPMAt110in, kHoodAt110in));
    }

    private ShotTable() {}

    public static Shot liveShot(double constantsRpm, double hood) {
        return new Shot(constantsRpm + kLiveRpmOffset, hood);
    }

    /** Compiled live table only — used by unit tests and as NT fallback. */
    public static Shot compiled(Distance distance) {
        double inches = 75.125;
        if (distance != null) {
            final double raw = distance.in(Inches);
            if (Double.isFinite(raw)) {
                inches = MathUtil.clamp(raw, 47.0, 110.0);
            }
        }
        return kCompiledTable.get(Inches.of(inches));
    }

    public static Shot compiledInches(double inches) {
        return compiled(Inches.of(inches));
    }

    public static Shot get(Distance distance) {
        return compiled(distance);
    }

    public static Shot getInches(double inches) {
        return get(Inches.of(inches));
    }

    public static void publishNtDefaults() {
        final NetworkTable table = NetworkTableInstance.getDefault().getTable("ShotTable");
        putPoint(table, "47", 47.0, compiledInches(47.0));
        putPoint(table, "50", 50.0, compiledInches(50.0));
        putPoint(table, "75", 75.125, compiledInches(75.125));
        putPoint(table, "84", 84.0, compiledInches(84.0));
        putPoint(table, "92", 92.0, compiledInches(92.0));
        putPoint(table, "100", 100.0, compiledInches(100.0));
        putPoint(table, "110", 110.0, compiledInches(110.0));
    }

    private static void putPoint(NetworkTable table, String key, double inches, Shot shot) {
        table.getEntry(key + "/in").setDouble(inches);
        table.getEntry(key + "/rpm").setDouble(shot.shooterRPM);
        table.getEntry(key + "/hood").setDouble(shot.hoodPosition);
        SmartDashboard.putNumber("ShotTable " + key + " RPM", shot.shooterRPM);
        SmartDashboard.putNumber("ShotTable " + key + " Hood", shot.hoodPosition);
    }

    private static Shot interpolateWithNt(double inches) {
        final NetworkTable table = NetworkTableInstance.getDefault().getTable("ShotTable");
        final String[] keys = {"47", "50", "75", "84", "92", "100", "110"};
        double lowerIn = Double.NEGATIVE_INFINITY;
        double upperIn = Double.POSITIVE_INFINITY;
        Shot lower = null;
        Shot upper = null;
        for (String key : keys) {
            final double pointIn = table.getEntry(key + "/in").getDouble(Double.NaN);
            if (!Double.isFinite(pointIn)) {
                continue;
            }
            final Shot shot = new Shot(
                table.getEntry(key + "/rpm").getDouble(compiledInches(pointIn).shooterRPM),
                table.getEntry(key + "/hood").getDouble(compiledInches(pointIn).hoodPosition)
            );
            if (pointIn <= inches && pointIn >= lowerIn) {
                lowerIn = pointIn;
                lower = shot;
            }
            if (pointIn >= inches && pointIn <= upperIn) {
                upperIn = pointIn;
                upper = shot;
            }
        }
        if (lower == null && upper == null) {
            return compiledInches(inches);
        }
        if (lower == null) {
            return upper;
        }
        if (upper == null) {
            return lower;
        }
        if (Math.abs(upperIn - lowerIn) < 1e-6) {
            return lower;
        }
        final double t = (inches - lowerIn) / (upperIn - lowerIn);
        return new Shot(
            lower.shooterRPM + t * (upper.shooterRPM - lower.shooterRPM),
            lower.hoodPosition + t * (upper.hoodPosition - lower.hoodPosition)
        );
    }
}
