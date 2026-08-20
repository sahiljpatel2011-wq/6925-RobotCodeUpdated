package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import frc.robot.vision.ShotTable;
import frc.robot.vision.ShotTable.Shot;

class ShotTableTest {
    @Test
    void liveTablePoints() {
        assertShot(47.0, 3500.0, 0.00);
        assertShot(50.0, 3250.0, 0.02);
        assertShot(75.125, 3500.0, 0.05);
        assertShot(84.0, 3500.0, 0.15);
        assertShot(92.0, 3500.0, 0.30);
        assertShot(100.0, 3600.0, 0.45);
        assertShot(110.0, 3650.0, 0.50);
    }

    @Test
    void interpolatesBetweenFiftyAndSeventyFive() {
        final Shot mid = ShotTable.compiledInches(62.5625);
        assertEquals(3375.0, mid.shooterRPM, 1.0);
        assertTrueHoodBetween(mid.hoodPosition, 0.02, 0.05);
    }

    @Test
    void unusedConstantsRpmIsNotTheLiveShot() {
        final Shot at47 = ShotTable.compiledInches(47.0);
        assertEquals(3500.0, at47.shooterRPM, 1e-6);
        org.junit.jupiter.api.Assertions.assertNotEquals(3100.0, at47.shooterRPM);
    }

    @Test
    void clampsOutsideTableAndIgnoresNonFinite() {
        assertEquals(3500.0, ShotTable.compiledInches(10.0).shooterRPM, 1e-6);
        assertEquals(3650.0, ShotTable.compiledInches(200.0).shooterRPM, 1e-6);
        assertEquals(
            ShotTable.compiledInches(75.125).shooterRPM,
            ShotTable.compiled(edu.wpi.first.units.Units.Inches.of(Double.NaN)).shooterRPM,
            1e-6);
    }

    @Test
    void liveGetUsesCompiledTable() {
        assertEquals(
            ShotTable.compiledInches(75.125).shooterRPM,
            ShotTable.getInches(75.125).shooterRPM,
            1e-6);
        assertEquals(
            ShotTable.compiledInches(75.125).hoodPosition,
            ShotTable.getInches(75.125).hoodPosition,
            1e-6);
    }

    private static void assertShot(double inches, double rpm, double hood) {
        final Shot shot = ShotTable.compiledInches(inches);
        assertEquals(rpm, shot.shooterRPM, 1e-6, "RPM at " + inches + " in");
        assertEquals(hood, shot.hoodPosition, 1e-6, "hood at " + inches + " in");
    }

    private static void assertTrueHoodBetween(double hood, double lo, double hi) {
        org.junit.jupiter.api.Assertions.assertTrue(hood > lo && hood < hi, "hood " + hood);
    }
}
