package frc.robot;

import java.util.ArrayList;
import java.util.List;

/**
 * Driver-station checklist only. Does not change drive, aim, RPM, or bindings.
 */
public final class MatchReady {
    public static final double kMinBatteryVolts = 12.0;

    private MatchReady() {}

    public static String checklist(
        boolean limelightStreaming,
        boolean pipelineAprilTags,
        boolean visionSeeded,
        boolean allianceKnown,
        boolean driverConnected,
        boolean operatorConnected,
        double batteryVolts,
        String autoName
    ) {
        final List<String> missing = new ArrayList<>();
        if (!limelightStreaming) {
            missing.add("Limelight");
        }
        if (!pipelineAprilTags) {
            missing.add("pipeline 0");
        }
        if (!visionSeeded) {
            missing.add("pose seed");
        }
        if (!allianceKnown) {
            missing.add("alliance");
        }
        if (!driverConnected) {
            missing.add("Xbox 0");
        }
        if (!operatorConnected) {
            missing.add("X3D 1");
        }
        if (!Double.isFinite(batteryVolts) || batteryVolts < kMinBatteryVolts) {
            missing.add("battery");
        }
        if (!autoSelected(autoName)) {
            missing.add("auto");
        }
        if (missing.isEmpty()) {
            return "READY";
        }
        return "WAIT: " + String.join(", ", missing);
    }

    public static boolean isReady(String checklist) {
        return "READY".equals(checklist);
    }

    /** True when the DS chooser is a real PathPlanner auto, not the empty option. */
    public static boolean autoSelected(String autoName) {
        if (autoName == null) {
            return false;
        }
        final String name = autoName.trim();
        if (name.isEmpty() || "None".equals(name)) {
            return false;
        }
        return !"InstantCommand".equals(name) && !"ParallelCommandGroup".equals(name);
    }

    /**
     * At events, keep the fused pose through auto → teleop. In the pit, reseed
     * every disable so moving the robot by hand still locates.
     */
    public static boolean clearVisionSeedOnDisable(boolean fmsAttached, boolean alreadySeeded) {
        return !(fmsAttached && alreadySeeded);
    }
}
