package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Compiled kill-switch defaults for 6925. Getters always return these values
 * so Shuffleboard cannot change aim, vision, or the shot table mid-match.
 */
public final class FeatureFlags {
    public static final boolean DEFAULT_VISION_IN_AUTO = true;
    public static final boolean DEFAULT_HUB_TAG_AIM_FILTER = true;
    public static final boolean DEFAULT_AUTOSHOOT_FEED = false;
    public static final boolean DEFAULT_NT_SHOT_TABLE = false;
    public static final boolean DEFAULT_VISION_POSE_FUSE = true;

    private FeatureFlags() {}

    public static void publishDefaults() {
        final NetworkTable table = NetworkTableInstance.getDefault().getTable("FeatureFlags");
        table.getEntry("visionInAuto").setBoolean(DEFAULT_VISION_IN_AUTO);
        table.getEntry("hubTagAimFilter").setBoolean(DEFAULT_HUB_TAG_AIM_FILTER);
        table.getEntry("autoshootFeed").setBoolean(DEFAULT_AUTOSHOOT_FEED);
        table.getEntry("ntShotTable").setBoolean(DEFAULT_NT_SHOT_TABLE);
        table.getEntry("visionPoseFuse").setBoolean(DEFAULT_VISION_POSE_FUSE);
        SmartDashboard.putBoolean("Flag visionInAuto", DEFAULT_VISION_IN_AUTO);
        SmartDashboard.putBoolean("Flag hubTagAimFilter", DEFAULT_HUB_TAG_AIM_FILTER);
        SmartDashboard.putBoolean("Flag autoshootFeed", DEFAULT_AUTOSHOOT_FEED);
        SmartDashboard.putBoolean("Flag visionPoseFuse", DEFAULT_VISION_POSE_FUSE);
    }

    public static boolean visionInAuto() {
        return DEFAULT_VISION_IN_AUTO;
    }

    public static boolean hubTagAimFilter() {
        return DEFAULT_HUB_TAG_AIM_FILTER;
    }

    public static boolean autoshootFeed() {
        return DEFAULT_AUTOSHOOT_FEED;
    }

    public static boolean ntShotTable() {
        return DEFAULT_NT_SHOT_TABLE;
    }

    public static boolean visionPoseFuse() {
        return DEFAULT_VISION_POSE_FUSE;
    }
}
