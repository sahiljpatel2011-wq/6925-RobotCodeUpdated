package frc.robot;

import java.util.Collections;
import java.util.LinkedHashSet;
import java.util.Set;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;

import com.pathplanner.lib.auto.NamedCommands;

/**
 * PathPlanner named-command names and aliases. Registration uses
 * {@code Commands.defer} so each schedule gets a fresh command instance.
 */
public final class NamedCommandRegistry {
    private static final Set<String> NAMES = new LinkedHashSet<>();

    public static final String SHOOT = "shoot";
    public static final String AUTO_SHOOT = "autoShoot";
    public static final String STOP_FEED = "StopFeed";
    public static final String STOP_INTAKE = "StopIntake";
    public static final String INTAKE_STOP = "intakeStop";
    public static final String INTAKE_FAST = "IntakeFast";
    public static final String INTAKE_FAST_ALIAS = "IntakeFast";
    public static final String INTAKE_MID = "IntakeMid";
    public static final String INTAKE_DEPLOY = "intakeDeploy";
    public static final String INTAKE_DEPLOY_ALIAS = "intakeDeploy";
    public static final String ADJUSTED_WIND_UP = "AdjustedWindUp";
    public static final String ADJUSTED_WIND_UP_ONCE = "AdjustedWindUpOnce";
    public static final String ADJUSTED_SHOOT_WHILE_MOVING = "AdjustedShootWhileMoving";
    public static final String HOPPER_DEPLOY = "hopperDeploy";
    public static final String HOPPER_DEPLOY_ALIAS = "hopperDeploy";
    public static final String HOOD_RESET = "hoodReset";
    public static final String HOOD_RESET_ALIAS = "hoodReset";

    private NamedCommandRegistry() {}

    public static void register(String name, Supplier<Command> factory, Subsystem... requirements) {
        NAMES.add(name);
        NamedCommands.registerCommand(
            name,
            Commands.defer(factory, Set.of(requirements))
        );
    }

    public static void registerNone(String name) {
        NAMES.add(name);
        NamedCommands.registerCommand(name, Commands.none());
    }

    public static boolean isRegistered(String name) {
        return NAMES.contains(name);
    }

    public static boolean aliasesMatch(String a, String b) {
        return NAMES.contains(a) && NAMES.contains(b);
    }

    public static Set<String> names() {
        return Collections.unmodifiableSet(NAMES);
    }

    public static void record(String name) {
        NAMES.add(name);
    }

    public static void resetForTest() {
        NAMES.clear();
    }
}
