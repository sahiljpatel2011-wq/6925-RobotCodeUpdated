package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

class LandmarksTest {
    private static final double kRebuiltLengthInches = 54.269 * 12.0;
    private static final double kRebuiltWidthInches = 26.474 * 12.0;

    @Test
    void hubsMatch2026RebuiltField() {
        assertEquals(182.105, metersToInches(Landmarks.kBlueHub.getX()), 0.05);
        assertEquals(469.115, metersToInches(Landmarks.kRedHub.getX()), 0.05);
        assertEquals(158.845, metersToInches(Landmarks.kBlueHub.getY()), 0.05);
        assertEquals(158.845, metersToInches(Landmarks.kRedHub.getY()), 0.05);
        assertEquals(kRebuiltWidthInches / 2.0, 158.845, 0.05);
        assertEquals(kRebuiltLengthInches, 182.105 + 469.115, 0.05);
    }

    private static double metersToInches(double meters) {
        return meters / 0.0254;
    }
}
