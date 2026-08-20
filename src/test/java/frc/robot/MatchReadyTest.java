package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class MatchReadyTest {
    @Test
    void readyWhenEveryCheckPasses() {
        final String status = MatchReady.checklist(
            true, true, true, true, true, true, 12.6, "M-S");
        assertEquals("READY", status);
        assertTrue(MatchReady.isReady(status));
    }

    @Test
    void listsMissingItemsWithoutChangingRobotBehavior() {
        final String status = MatchReady.checklist(
            false, false, false, false, false, false, 11.0, "None");
        assertTrue(status.startsWith("WAIT:"));
        assertTrue(status.contains("Limelight"));
        assertTrue(status.contains("pipeline 0"));
        assertTrue(status.contains("pose seed"));
        assertTrue(status.contains("alliance"));
        assertTrue(status.contains("Xbox 0"));
        assertTrue(status.contains("X3D 1"));
        assertTrue(status.contains("battery"));
        assertTrue(status.contains("auto"));
        assertFalse(MatchReady.isReady(status));
    }

    @Test
    void autoSelectedIgnoresEmptyChooserNames() {
        assertTrue(MatchReady.autoSelected("M-S"));
        assertFalse(MatchReady.autoSelected("None"));
        assertFalse(MatchReady.autoSelected("InstantCommand"));
        assertFalse(MatchReady.autoSelected(""));
        assertFalse(MatchReady.autoSelected(null));
    }

    @Test
    void matchKeepsSeedThroughDisablePitReseeds() {
        assertFalse(MatchReady.clearVisionSeedOnDisable(true, true));
        assertTrue(MatchReady.clearVisionSeedOnDisable(true, false));
        assertTrue(MatchReady.clearVisionSeedOnDisable(false, true));
        assertTrue(MatchReady.clearVisionSeedOnDisable(false, false));
    }
}
