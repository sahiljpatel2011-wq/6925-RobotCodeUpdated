package frc.robot;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import frc.robot.vision.FuelDetect;

class FuelDetectTest {
    @Test
    void acceptsFloorFuelBlob() {
        assertTrue(FuelDetect.isFieldFuel(0, 0.8, -6.0));
        assertTrue(FuelDetect.isFieldFuel(0, 0.15, 0.0));
    }

    @Test
    void rejectsHopperCeilingAndNoise() {
        assertFalse(FuelDetect.isFieldFuel(0, 0.8, 12.0), "hopper/ceiling");
        assertFalse(FuelDetect.isFieldFuel(0, 0.05, -4.0), "too small");
        assertFalse(FuelDetect.isFieldFuel(1, 1.0, -4.0), "wrong class");
        assertFalse(FuelDetect.isFieldFuel(0, 0.8, -30.0), "under camera");
        assertFalse(FuelDetect.isFieldFuel(0, 15.0, -8.0), "hopper blob too large");
    }
}
