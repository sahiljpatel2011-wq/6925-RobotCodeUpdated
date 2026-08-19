package frc.robot;

import static edu.wpi.first.units.Units.Inches;

import java.util.Optional;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Landmarks {
    public static final Translation2d kBlueHub =
        new Translation2d(Inches.of(182.105), Inches.of(158.845));
    public static final Translation2d kRedHub =
        new Translation2d(Inches.of(469.115), Inches.of(158.845));

    public static Optional<Alliance> alliance() {
        return DriverStation.getAlliance();
    }

    public static boolean isBlueAlliance() {
        return alliance().filter(a -> a == Alliance.Blue).isPresent();
    }

    public static boolean isAllianceKnown() {
        return alliance().isPresent();
    }

    /**
     * Hub XY for the current alliance. Empty when FMS has not published yet
     * so callers do not silently aim at the red hub.
     */
    public static Optional<Translation2d> targetPositionOptional() {
        final Optional<Alliance> alliance = alliance();
        if (alliance.isEmpty()) {
            SmartDashboard.putString("Alliance", "?");
            return Optional.empty();
        }
        if (alliance.get() == Alliance.Blue) {
            SmartDashboard.putString("Alliance", "Blue");
            return Optional.of(kBlueHub);
        }
        SmartDashboard.putString("Alliance", "Red");
        return Optional.of(kRedHub);
    }

    /** Last-resort hub used only when a Distance is required and alliance is unknown. */
    public static Translation2d targetPosition() {
        return targetPositionOptional().orElse(kBlueHub);
    }
}
