package frc.robot;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class FeatureFlagsTest {
    @Test
    void compiledDefaultsMatchToday() {
        assertFalse(FeatureFlags.DEFAULT_FUEL_ASSIST);
        assertFalse(FeatureFlags.DEFAULT_AUTOSHOOT_FEED);
        assertFalse(FeatureFlags.DEFAULT_NT_SHOT_TABLE);
        assertTrue(FeatureFlags.DEFAULT_HUB_TAG_AIM_FILTER);
        assertTrue(FeatureFlags.DEFAULT_VISION_POSE_FUSE);
        assertTrue(FeatureFlags.DEFAULT_VISION_IN_AUTO);
    }

    @Test
    void disablingFuelDoesNotEnableAutoshoot() {
        assertFalse(FeatureFlags.DEFAULT_FUEL_ASSIST && FeatureFlags.DEFAULT_AUTOSHOOT_FEED);
        assertFalse(FeatureFlags.DEFAULT_AUTOSHOOT_FEED);
    }
}
