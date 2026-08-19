package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import frc.robot.vision.HubAimMath;

class HubAimMathTest {
    @Test
    void hubIdsAcceptedAndRejected() {
        assertTrue(HubAimMath.isHubTag(2));
        assertTrue(HubAimMath.isHubTag(3));
        assertTrue(HubAimMath.isHubTag(8));
        assertTrue(HubAimMath.isHubTag(19));
        assertTrue(HubAimMath.isHubTag(27));
        assertFalse(HubAimMath.isHubTag(7));
        assertFalse(HubAimMath.isHubTag(15));
        assertTrue(HubAimMath.isAllianceHubTag(18, true));
        assertFalse(HubAimMath.isAllianceHubTag(8, true));
        assertTrue(HubAimMath.isAllianceHubTag(8, false));
        assertTrue(HubAimMath.isTrenchTag(1));
        assertTrue(HubAimMath.isTrenchTag(22));
    }

    @Test
    void eightInchLateralSigns() {
        assertEquals(-8.0, HubAimMath.lateralOffsetInches(8), 1e-9);
        assertEquals(-8.0, HubAimMath.lateralOffsetInches(24), 1e-9);
        assertEquals(-8.0, HubAimMath.lateralOffsetInches(3), 1e-9);
        assertEquals(-8.0, HubAimMath.lateralOffsetInches(19), 1e-9);
        assertEquals(8.0, HubAimMath.lateralOffsetInches(9), 1e-9);
        assertEquals(8.0, HubAimMath.lateralOffsetInches(11), 1e-9);
        assertEquals(8.0, HubAimMath.lateralOffsetInches(25), 1e-9);
        assertEquals(8.0, HubAimMath.lateralOffsetInches(27), 1e-9);
        assertEquals(0.0, HubAimMath.lateralOffsetInches(2), 1e-9);
        assertEquals(0.0, HubAimMath.lateralOffsetInches(18), 1e-9);
    }

    @Test
    void tanGuardRejectsNearZeroAndNinety() {
        assertTrue(HubAimMath.cameraToTagInches(-26.0).isEmpty());
        assertTrue(HubAimMath.cameraToTagInches(64.0).isEmpty());
        assertTrue(HubAimMath.cameraToTagInches(0.0).isPresent());
    }

    @Test
    void poseFallbackProducesYawErrorWhenTvFalse() {
        final double tx = HubAimMath.poseAimTxDegrees(2.0, 4.0, 0.0, 4.625, 4.035);
        assertTrue(Math.abs(tx) > 0.5, "pose fallback must still yaw toward the hub");
        assertTrue(HubAimMath.poseRangeInches(2.0, 4.0, 4.625, 4.035) > 50.0);
    }

    @Test
    void hubAimTxIsDefinedForCenteredTag() {
        assertTrue(HubAimMath.hubAimTxDegrees(2, 0.0, 0.0).isPresent());
        assertTrue(HubAimMath.hubRangeInches(2, 0.0, 0.0).getAsDouble() > 23.5);
        assertTrue(HubAimMath.hubAimTxDegrees(7, 0.0, 0.0).isEmpty());
    }
}
