package frc.robot;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import frc.robot.vision.VisionGates;

class VisionGatesTest {
    @Test
    void fuseRejectsOriginAndFarSingleTag() {
        assertFalse(VisionGates.isValidMegaTag2Fuse(1, 1.0, 2.0, 0.0, 0.0));
        assertFalse(VisionGates.isValidMegaTag2Fuse(1, 1.0, 4.5, 2.0, 4.0));
        assertFalse(VisionGates.isValidMegaTag2Fuse(2, 1.0, 8.0, 2.0, 4.0));
        assertTrue(VisionGates.isValidMegaTag2Fuse(1, 1.0, 3.0, 2.0, 4.0));
        assertTrue(VisionGates.isValidMegaTag2Fuse(2, 0.5, 5.0, 2.0, 4.0));
    }

    @Test
    void seedAllowsOneTagThatFuseWouldReject() {
        assertTrue(VisionGates.isValidMegaTag1Seed(1, 0.4, 5.5, 1.2, 4.0));
        assertFalse(VisionGates.isValidMegaTag2Fuse(1, 0.4, 5.5, 1.2, 4.0));
        assertFalse(VisionGates.isValidMegaTag1Seed(1, 0.4, 5.5, 0.0, 0.0));
        assertFalse(VisionGates.isValidMegaTag1Seed(0, 1.0, 2.0, 2.0, 4.0));
    }

    @Test
    void firstJumpFromOriginIsAccepted() {
        assertTrue(VisionGates.allowVisionJump(0.0, 0.0, 4.7, 1.0));
        assertFalse(VisionGates.allowVisionJump(2.0, 4.0, 4.7, 1.0));
        assertTrue(VisionGates.allowVisionJump(2.0, 4.0, 0.4, 1.0));
    }
}
