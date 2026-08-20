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
        assertTrue(HubAimMath.isHubTag(10));
        assertTrue(HubAimMath.isHubTag(26));
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
    void poseAimTxMatchesLimelightSign() {
        // Facing +X: +Y is left (negative tx), -Y is right (positive tx).
        assertTrue(HubAimMath.poseAimTxDegrees(0.0, 0.0, 0.0, 0.0, 1.0) < -45.0);
        assertTrue(HubAimMath.poseAimTxDegrees(0.0, 0.0, 0.0, 0.0, -1.0) > 45.0);
        assertEquals(0.0, HubAimMath.poseAimTxDegrees(0.0, 0.0, 0.0, 2.0, 0.0), 1e-6);
    }

    @Test
    void hubAimTxIsDefinedForCenteredTag() {
        assertTrue(HubAimMath.hubAimTxDegrees(2, 0.0, 0.0).isPresent());
        assertTrue(HubAimMath.hubRangeInches(2, 0.0, 0.0).getAsDouble() > 23.5);
        assertTrue(HubAimMath.hubAimTxDegrees(7, 0.0, 0.0).isEmpty());
    }

    @Test
    void areaWeightedHubAimBiasesTowardLargerTag() {
        final double leftTx = HubAimMath.hubAimTxDegrees(2, -4.0, 0.0).getAsDouble();
        final double rightTx = HubAimMath.hubAimTxDegrees(2, 4.0, 0.0).getAsDouble();
        final var equal = HubAimMath.areaWeightedHubAim(
            new int[] {2, 2},
            new double[] {-4.0, 4.0},
            new double[] {0.0, 0.0},
            new double[] {1.0, 1.0}
        ).orElseThrow();
        assertEquals((leftTx + rightTx) / 2.0, equal.txDegrees, 1e-6);
        assertEquals(2, equal.count);

        final var biased = HubAimMath.areaWeightedHubAim(
            new int[] {2, 2},
            new double[] {-4.0, 4.0},
            new double[] {0.0, 0.0},
            new double[] {0.2, 1.8}
        ).orElseThrow();
        assertTrue(biased.txDegrees > equal.txDegrees);
        assertEquals(2, biased.bestId);
    }

    @Test
    void hubTagIsUsedEvenWhenTyCannotGiveRange() {
        final var aim = HubAimMath.areaWeightedHubAim(
            new int[] {26},
            new double[] {4.0},
            new double[] {-26.0},
            new double[] {1.0}
        ).orElseThrow();
        assertEquals(4.0, aim.txDegrees, 1e-6);
        assertEquals(26, aim.bestId);
        assertTrue(Double.isNaN(aim.rangeInches));
        assertEquals(26, HubAimMath.preferredHubTagId(true));
        assertEquals(10, HubAimMath.preferredHubTagId(false));
    }

    @Test
    void bestHubIndexIgnoresTrenchAndPicksLargestHub() {
        assertEquals(
            1,
            HubAimMath.bestHubIndex(
                new int[] {7, 26, 18},
                new double[] {5.0, 1.2, 0.4},
                true,
                true));
        assertEquals(
            -1,
            HubAimMath.bestHubIndex(
                new int[] {7, 8},
                new double[] {2.0, 1.0},
                true,
                true));
        assertEquals(
            1,
            HubAimMath.bestHubIndex(
                new int[] {7, 8},
                new double[] {2.0, 1.0},
                false,
                true));
    }

    @Test
    void aimAssistTurnsBotAndHoldsBriefly() {
        assertEquals(-0.15 * 0.5, HubAimMath.aimAssistOmega(0.5, 0.15), 1e-9);
        assertEquals(-0.15 * 4.0, HubAimMath.aimAssistOmega(4.0, 0.15), 1e-9);
        assertTrue(HubAimMath.keepLastAim(1.10, 1.00, 0.15));
        assertFalse(HubAimMath.keepLastAim(1.20, 1.00, 0.15));
        assertFalse(HubAimMath.keepLastAim(1.00, -1.0, 0.15));
        assertTrue(HubAimMath.isOnTarget(0.0));
        assertFalse(HubAimMath.isOnTarget(2.0));
        assertFalse(HubAimMath.autoFeed(false, true, false));
        assertFalse(HubAimMath.autoFeed(true, false, false));
        assertTrue(HubAimMath.autoFeed(true, true, false));
        assertTrue(HubAimMath.autoFeed(false, false, true), "keep feeding after lock");
    }

    @Test
    void movingShotUsesLookAheadOnLimelightRange() {
        assertEquals(
            100.0,
            HubAimMath.movingShotInches(100.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 0.25),
            1e-6);
        final double towardHub = HubAimMath.movingShotInches(100.0, 0.0, 0.0, 2.0, 0.0, 1.0, 0.0, 0.25);
        assertTrue(towardHub < 100.0);
        assertEquals(100.0 - 0.25 / 0.0254, towardHub, 1e-6);
        assertEquals(
            0.0,
            HubAimMath.movingLeadTxDegrees(0.0, 0.0, 0.0, 2.0, 0.0, 1.0, 0.0, 0.25),
            1e-6);
        final double strafeLead = HubAimMath.movingLeadTxDegrees(
            0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 1.0, 0.25);
        assertTrue(strafeLead > 1.0, "after strafing left the hub is to the right of the nose");
    }

    @Test
    void tagGeometryDoesNotDoubleCountHubHeading() {
        final double robotX = 2.55;
        final double robotY = 4.04;
        final double hubX = 4.625;
        final double hubY = 4.035;
        final double[] tag = HubAimMath.tagXyMeters(robotX, robotY, hubX, hubY);
        final double tagTx = HubAimMath.poseAimTxDegrees(robotX, robotY, 0.0, tag[0], tag[1]);
        final double cameraToTag = Math.hypot(tag[0] - robotX, tag[1] - robotY) / 0.0254;
        final double ty = HubAimMath.tyDegreesFromCameraToTagInches(cameraToTag);
        final double hubTx = HubAimMath.hubAimTxDegrees(2, tagTx, ty).orElseThrow();
        final double hubHeading = HubAimMath.poseAimTxDegrees(robotX, robotY, 0.0, hubX, hubY);
        assertEquals(hubHeading, hubTx, 3.0, "sim tag tx+ty must aim at hub, not past it");
    }
}
