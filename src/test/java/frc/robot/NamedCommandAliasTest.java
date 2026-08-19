package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class NamedCommandAliasTest {
    @Test
    void intakeStopAliasesStopIntake() {
        assertEquals(NamedCommandRegistry.INTAKE_STOP, "intakeStop");
        assertEquals(NamedCommandRegistry.STOP_INTAKE, "StopIntake");
        assertTrue(!NamedCommandRegistry.INTAKE_STOP.equals(NamedCommandRegistry.STOP_INTAKE));
    }

    @Test
    void registryListsRequiredAliases() {
        NamedCommandRegistry.resetForTest();
        NamedCommandRegistry.record(NamedCommandRegistry.STOP_INTAKE);
        NamedCommandRegistry.record(NamedCommandRegistry.INTAKE_STOP);
        NamedCommandRegistry.record(NamedCommandRegistry.HOPPER_DEPLOY);
        NamedCommandRegistry.record(NamedCommandRegistry.HOPPER_DEPLOY_ALIAS);
        NamedCommandRegistry.record("hoodReset");
        NamedCommandRegistry.record("hoodReset");
        assertTrue(NamedCommandRegistry.aliasesMatch(
            NamedCommandRegistry.INTAKE_STOP, NamedCommandRegistry.STOP_INTAKE));
        assertTrue(NamedCommandRegistry.isRegistered("intakeStop"));
        assertTrue(NamedCommandRegistry.isRegistered("StopIntake"));
        assertTrue(NamedCommandRegistry.isRegistered("hopperDeploy"));
        assertTrue(NamedCommandRegistry.isRegistered("hopperDeploy"));
        assertTrue(NamedCommandRegistry.isRegistered("hoodReset"));
        assertTrue(NamedCommandRegistry.isRegistered("hoodReset"));
    }
}
