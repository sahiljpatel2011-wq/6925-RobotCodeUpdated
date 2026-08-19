package frc.robot;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * NT kill-switches for 6925. Defaults match current teleop unless a must-fix
 * requires the new behavior (hub-tag aim, XY-only vision). Optional features
 * stay off until enabled from the DS.
 */
public final class FeatureFlags {
    public static final boolean DEFAULT_VISION_IN_AUTO = true;
    public static final boolean DEFAULT_HUB_TAG_AIM_FILTER = true;
    public static final boolean DEFAULT_FUEL_ASSIST = false;
    public static final boolean DEFAULT_AUTOSHOOT_FEED = false;
    public static final boolean DEFAULT_NT_SHOT_TABLE = false;
    public static final boolean DEFAULT_VISION_POSE_FUSE = true;

    private static final NetworkTable TABLE =
        NetworkTableInstance.getDefault().getTable("FeatureFlags");

    private static final BooleanSubscriber visionInAuto =
        TABLE.getBooleanTopic("visionInAuto").subscribe(DEFAULT_VISION_IN_AUTO);
    private static final BooleanSubscriber hubTagAimFilter =
        TABLE.getBooleanTopic("hubTagAimFilter").subscribe(DEFAULT_HUB_TAG_AIM_FILTER);
    private static final BooleanSubscriber fuelAssist =
        TABLE.getBooleanTopic("fuelAssist").subscribe(DEFAULT_FUEL_ASSIST);
    private static final BooleanSubscriber autoshootFeed =
        TABLE.getBooleanTopic("autoshootFeed").subscribe(DEFAULT_AUTOSHOOT_FEED);
    private static final BooleanSubscriber ntShotTable =
        TABLE.getBooleanTopic("ntShotTable").subscribe(DEFAULT_NT_SHOT_TABLE);
    private static final BooleanSubscriber visionPoseFuse =
        TABLE.getBooleanTopic("visionPoseFuse").subscribe(DEFAULT_VISION_POSE_FUSE);

    private static final BooleanPublisher visionInAutoPub =
        TABLE.getBooleanTopic("visionInAuto").publish();
    private static final BooleanPublisher hubTagAimFilterPub =
        TABLE.getBooleanTopic("hubTagAimFilter").publish();
    private static final BooleanPublisher fuelAssistPub =
        TABLE.getBooleanTopic("fuelAssist").publish();
    private static final BooleanPublisher autoshootFeedPub =
        TABLE.getBooleanTopic("autoshootFeed").publish();
    private static final BooleanPublisher ntShotTablePub =
        TABLE.getBooleanTopic("ntShotTable").publish();
    private static final BooleanPublisher visionPoseFusePub =
        TABLE.getBooleanTopic("visionPoseFuse").publish();

    private FeatureFlags() {}

    public static void publishDefaults() {
        visionInAutoPub.set(DEFAULT_VISION_IN_AUTO);
        hubTagAimFilterPub.set(DEFAULT_HUB_TAG_AIM_FILTER);
        fuelAssistPub.set(DEFAULT_FUEL_ASSIST);
        autoshootFeedPub.set(DEFAULT_AUTOSHOOT_FEED);
        ntShotTablePub.set(DEFAULT_NT_SHOT_TABLE);
        visionPoseFusePub.set(DEFAULT_VISION_POSE_FUSE);
        SmartDashboard.putBoolean("Flag visionInAuto", DEFAULT_VISION_IN_AUTO);
        SmartDashboard.putBoolean("Flag hubTagAimFilter", DEFAULT_HUB_TAG_AIM_FILTER);
        SmartDashboard.putBoolean("Flag fuelAssist", DEFAULT_FUEL_ASSIST);
        SmartDashboard.putBoolean("Flag autoshootFeed", DEFAULT_AUTOSHOOT_FEED);
        SmartDashboard.putBoolean("Flag visionPoseFuse", DEFAULT_VISION_POSE_FUSE);
    }

    public static boolean visionInAuto() {
        return visionInAuto.get();
    }

    public static boolean hubTagAimFilter() {
        return hubTagAimFilter.get();
    }

    public static boolean fuelAssist() {
        return fuelAssist.get();
    }

    public static boolean autoshootFeed() {
        return autoshootFeed.get();
    }

    public static boolean ntShotTable() {
        return ntShotTable.get();
    }

    public static boolean visionPoseFuse() {
        return visionPoseFuse.get();
    }

    public static void setAutoshootFeed(boolean enabled) {
        autoshootFeedPub.set(enabled);
    }
}
