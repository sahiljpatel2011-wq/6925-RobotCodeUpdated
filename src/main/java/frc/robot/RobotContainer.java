// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/*
 * =========================================================================
 *                     FRC TEAM 6925 - ROBOT OVERVIEW
 * =========================================================================
 *
 * This is a WCP CC baseplate clone on SDS MK4i, scoring FUEL by shooting
 * with three independent Kraken flywheels. It is not a dump-bin robot and
 * does not climb. Too tall for the trench — use bump ramps.
 *
 * DRIVETRAIN (CommandSwerveDrivetrain)
 *   - Phoenix 6 Tuner X MK4i, field-centric open-loop voltage
 *   - Xbox port 0: left stick translate (squared + slew), right stick rotate (^1.5)
 *   - Default speed 0.75; B toggles 1.0; right trigger toggles 1/5
 *   - Left bumper reseeds field-centric heading
 *   - Left trigger points wheels to 0° for 0.5 s
 *   - A = X-brake
 *   - Right bumper = hub aim + live-table shooter wind-up (keep translating)
 *   - Y = full-field pass aim (trench tags, 15° inward) + 5450 RPM
 *   - Drive supply 35 A — do not raise. SysId is disabled/test only.
 *
 * SHOOTER (ShooterSubsys) — 3 independent TalonFX, not followers
 *   - CAN 8 / 9 / 10, VelocityVoltage, Coast
 *   - Idle hold 3000 RPM between volleys; last shot RPM is held while feeding
 *   - Live table (Constants + 150 RPM): 47"→3500/0.00 … 110"→3650/0.50
 *   - "Shooter At Speed" = all three columns within 300 RPM, false at idle
 *
 * HOOD (HoodSubsys) — PWM 0 left, PWM 1 right, 0.01–0.77
 *
 * FEEDER (FeederSubsys) — CAN 51 feeder + CAN 11 fuel
 *   - Separate subsystem so button 1 can feed while bumper holds flywheels
 *
 * INTAKE (IntakeSubsys) — CAN 45 roller, CAN 50 rotator
 *   - Button 2 intake+oscillate; 12 retract+oscillate; 6 deploy; 4 retract
 *   - Hat down = reverse jam
 *
 * CLIMBER — removed. hopperDeploy / jolt / Climb* named commands are no-ops.
 *
 * LIMELIGHT ("limelight")
 *   - 1.46" behind center, 25.39" up, 26° pitch. Fiducial offset -0.5842 m
 *     (keep until range day). Pipeline 0 AprilTag / MegaTag2 XY-only.
 *   - Pipeline 1 Fuel B1 (hold operator 3). Alliance hub tags only for aim.
 *
 * OPERATOR (X3D port 1)
 *   1  = Shoot (feeder + intake bounce) + hold 1/5 drive
 *   2  = Intake oscillate + hold 1/2 drive
 *   3  = Fuel assist (pipeline 1, default flag OFF)
 *   4  = Retract    5 = AutoshootFeed (default OFF)
 *   6  = Deploy     7 = closer hub windup (3350 / 0.0)
 *   8  = Pass windup (3500 / 0.75)
 *   9  = Close windup    10 = snap wheels
 *   11 = Test hood       12 = retract oscillate
 *   Hat down = reverse; hat left = exposure tune (disabled only)
 *
 * AUTONOMOUS
 *   PathPlanner AutoBuilder. Sequential named commands must finish.
 *   Aliases: intakeStop=StopIntake, hopperDeploy=hopperDeploy,
 *   hoodReset=hoodReset, intakeDeploy=intakeDeploy.
 *   Robot is too tall for trench; bump-ramp autos only. No climb auto.
 *
 * MOTOR CAN IDs
 *   8/9/10 shooter, 11 fuel feed, 45 intake roller, 50 rotator, 51 feeder
 *   Swerve in TunerConstants (do not change 35 A drive limits)
 * =========================================================================
 */

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.lib.util.CommandX3DController;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.FeederSubsys;
import frc.robot.subsystems.HoodSubsys;
import frc.robot.subsystems.IntakeSubsys;
import frc.robot.subsystems.LimelightSubsys;
import frc.robot.subsystems.ShooterSubsys;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private double MaxAngularRate = RotationsPerSecond.of(1.5).in(RadiansPerSecond);

    private final SlewRateLimiter xLimiter = new SlewRateLimiter(1.5, -100, 0);
    private final SlewRateLimiter yLimiter = new SlewRateLimiter(1.5, -100, 0);

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.05).withRotationalDeadband(MaxAngularRate * 0.075)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);
    private final SendableChooser<Command> autoChooser;
    private String cachedAutoName = "None";
    private boolean visionSeeded = false;

    private final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandX3DController operator = new CommandX3DController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private final ShooterSubsys shooter = new ShooterSubsys();
    private final IntakeSubsys intake = new IntakeSubsys();
    private final FeederSubsys feeder = new FeederSubsys();
    private final HoodSubsys hood = new HoodSubsys();
    private final LimelightSubsys limelight = new LimelightSubsys(
        "limelight",
        () -> drivetrain.getState().Pose,
        () -> drivetrain.getState().Pose.getRotation().getDegrees()
    );

    public RobotContainer() {
        RobotCommands.init(shooter, feeder, hood, intake, drivetrain, limelight);
        configureBindings();
        registerNamedCommands();
        drivetrain.registerTelemetry(logger::telemeterize);

        autoChooser = AutoBuilder.buildAutoChooser("M-S");
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    private void registerNamedCommands() {
        NamedCommandRegistry.register("shoot", RobotCommands::Shoot, feeder, intake);
        NamedCommandRegistry.register("shootHold", RobotCommands::Shoot, feeder, intake);
        NamedCommandRegistry.register("autoShoot", () -> RobotCommands.autoShoot(4), feeder, intake);
        NamedCommandRegistry.register("StopFeed", RobotCommands::stopFeed, feeder);
        NamedCommandRegistry.register("windUp", RobotCommands::windUpOnce, shooter, hood);
        NamedCommandRegistry.register("windUpOnce", RobotCommands::windUpOnce, shooter, hood);
        NamedCommandRegistry.register("autoWindUp", RobotCommands::autoWindUp, shooter, hood);
        NamedCommandRegistry.register("autoWindUpClose", RobotCommands::autoWindUpClose, shooter, hood);
        NamedCommandRegistry.register("autoWindUpCloser", RobotCommands::autoWindUpCloser, shooter, hood);
        NamedCommandRegistry.register("AdjustedWindUp", RobotCommands::adjustedWindUp, shooter, hood);
        NamedCommandRegistry.register("AdjustedShootWhileMoving", RobotCommands::adjustedShootWhileMoving, shooter, hood, feeder);
        NamedCommandRegistry.register("AdjustedWindUpOnce", RobotCommands::adjustedWindUpOnce, shooter, hood);
        NamedCommandRegistry.register("IntakeMid", RobotCommands::intakeMid, intake);
        NamedCommandRegistry.register("IntakeFast", RobotCommands::intakeFast, intake);
        NamedCommandRegistry.register("IntakeFast", RobotCommands::intakeFast, intake);
        NamedCommandRegistry.register("StopIntake", RobotCommands::stopIntake, intake);
        NamedCommandRegistry.register("intakeStop", RobotCommands::stopIntake, intake);
        NamedCommandRegistry.register("intakeDeploy", () -> intake.goToPositionCommand(-14.5), intake);
        NamedCommandRegistry.register("intakeDeploy", () -> intake.goToPositionCommand(-14.5), intake);
        NamedCommandRegistry.register("intakeBounce", Commands::none);
        NamedCommandRegistry.register("hoodReset", () -> Commands.runOnce(() -> hood.setPosition(0)), hood);
        NamedCommandRegistry.register("hoodReset", () -> Commands.runOnce(() -> hood.setPosition(0)), hood);

        NamedCommandRegistry.registerNone("jolt");
        NamedCommandRegistry.registerNone("ClimbUp");
        NamedCommandRegistry.registerNone("ClimbDown");
        NamedCommandRegistry.registerNone("climbDown");
        NamedCommandRegistry.registerNone("StopClimber");
        NamedCommandRegistry.registerNone("hopperDeploy");
        NamedCommandRegistry.registerNone("hopperDeploy");
        NamedCommandRegistry.registerNone("VisionUpdate");
    }

    private double slewedForward() {
        double leftY = joystick.getLeftY();
        double leftX = joystick.getLeftX();
        boolean translationDead = Math.abs(leftY) < 0.05 && Math.abs(leftX) < 0.05;
        if (translationDead) {
            xLimiter.reset(0);
            yLimiter.reset(0);
            return 0;
        }
        double squaredY = -Math.copySign(leftY * leftY, leftY);
        return xLimiter.calculate(squaredY) * MaxSpeed * drivetrain.getCurrentSpeedMulti();
    }

    private double slewedStrafe() {
        double leftY = joystick.getLeftY();
        double leftX = joystick.getLeftX();
        boolean translationDead = Math.abs(leftY) < 0.05 && Math.abs(leftX) < 0.05;
        if (translationDead) {
            return 0;
        }
        double squaredX = -Math.copySign(leftX * leftX, leftX);
        return yLimiter.calculate(squaredX) * MaxSpeed * drivetrain.getCurrentSpeedMulti();
    }

    private void configureBindings() {
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() -> {
                double leftY = joystick.getLeftY();
                double leftX = joystick.getLeftX();
                double rightX = joystick.getRightX();
                boolean translationDead = Math.abs(leftY) < 0.05 && Math.abs(leftX) < 0.05;
                if (translationDead) {
                    xLimiter.reset(0);
                    yLimiter.reset(0);
                }
                double squaredY = translationDead ? 0 : -Math.copySign(leftY * leftY, leftY);
                double squaredX = translationDead ? 0 : -Math.copySign(leftX * leftX, leftX);
                double sqrtRot = -Math.copySign(Math.pow(Math.abs(rightX), 1.5), rightX);
                double slewedY = translationDead ? 0 : xLimiter.calculate(squaredY);
                double slewedX = translationDead ? 0 : yLimiter.calculate(squaredX);
                double transScale = shooter.isSpooling() ? 0.75 : 1.0;
                return drive.withVelocityX(slewedY * MaxSpeed * drivetrain.getCurrentSpeedMulti() * transScale)
                    .withVelocityY(slewedX * MaxSpeed * drivetrain.getCurrentSpeedMulti() * transScale)
                    .withRotationalRate(sqrtRot * MaxAngularRate);
            })
        );

        joystick.leftTrigger().onTrue(
            drivetrain.applyRequest(() -> point.withModuleDirection(new Rotation2d(0)))
                .withTimeout(0.5));
        joystick.rightTrigger().onTrue(drivetrain.toggleSpeedMulti(1.0 / 5.0));
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().onTrue(drivetrain.toggleSpeedMulti(1.0));

        var sysIdMode = RobotModeTriggers.disabled().or(RobotModeTriggers.test());
        joystick.back().and(joystick.y()).and(sysIdMode).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).and(sysIdMode).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).and(sysIdMode).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).and(sysIdMode).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        joystick.povUp().whileTrue(RobotCommands.windUp75());
        joystick.povDown().whileTrue(intake.slowRotateCommand(.025));
        joystick.povLeft().whileTrue(intake.creepRotateCommand(-1));
        joystick.povRight().whileTrue(intake.creepRotateCommand(1));

        joystick.rightBumper().whileTrue(
            RobotCommands.aimAndWindUp(this::slewedForward, this::slewedStrafe, MaxSpeed)
        );
        joystick.y().whileTrue(
            RobotCommands.aimAndPassFullField(this::slewedForward, this::slewedStrafe, MaxSpeed)
        );

        operator.button(1).whileTrue(RobotCommands.Shoot());
        operator.button(11).whileTrue(RobotCommands.windUpTest());
        operator.button(9).whileTrue(RobotCommands.windUpClose());
        operator.button(2).whileTrue(intake.intakeWithOscillateCommand(IntakeSubsys.IntakeSpeed.INTAKE_FAST));
        operator.button(2).whileTrue(drivetrain.holdSpeedMulti(1.0 / 2.0));
        operator.button(1).whileTrue(drivetrain.holdSpeedMulti(1.0 / 5.0));
        operator.button(12).whileTrue(intake.retractWithOscillateCommand(IntakeSubsys.IntakeSpeed.INTAKE_FAST));
        operator.button(7).whileTrue(RobotCommands.windUpCloser());
        operator.button(6).onTrue(intake.goToPositionSlowCommand(-14.20, 0.3));
        operator.button(4).onTrue(intake.goToPositionSlowCommand(-0.14423828125, 0.2));
        operator.button(10).whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(0))));
        operator.pov(180).whileTrue(RobotCommands.reverseAll());
        operator.pov(270).and(RobotModeTriggers.disabled()).onTrue(RobotCommands.autoTuneExposure());
        operator.button(8).whileTrue(RobotCommands.windUpPass());
        operator.button(3).whileTrue(
            RobotCommands.fuelAssist(this::slewedForward, this::slewedStrafe, MaxSpeed)
        );
        operator.button(5).whileTrue(RobotCommands.autoshootFeed(joystick.getHID()));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public void updateDashboard() {
        Command selected = autoChooser.getSelected();
        String name = selected != null ? selected.getName() : "None";
        if (!name.equals(cachedAutoName)) {
            cachedAutoName = name;
        }
        SmartDashboard.putString("Selected Auto", cachedAutoName);
        SmartDashboard.putBoolean("VisionSeeded", visionSeeded);
        RobotCommands.updateHud();
    }

    private static final double kMaxVisionJumpTeleopMeters = 1.0;
    private static final double kMaxVisionJumpAutoMeters = 2.0;
    private static final double kMaxYawRateDegPerSec = 360.0;

    public void updateVision() {
        if (limelight == null) {
            return;
        }
        if (DriverStation.isAutonomous() && !FeatureFlags.visionInAuto()) {
            return;
        }
        limelight.getMeasurement().ifPresent(measurement -> {
            final Pose2d currentPose = drivetrain.getState().Pose;
            final double jump = currentPose.getTranslation()
                .getDistance(measurement.poseEstimate.pose.getTranslation());
            final double maxJump = DriverStation.isAutonomous()
                ? kMaxVisionJumpAutoMeters
                : kMaxVisionJumpTeleopMeters;
            final double yawRate = Math.abs(drivetrain.getPigeon2().getAngularVelocityZWorld().getValueAsDouble());
            if (jump > maxJump || yawRate > kMaxYawRateDegPerSec) {
                return;
            }
            drivetrain.addVisionMeasurement(
                measurement.poseEstimate.pose,
                measurement.poseEstimate.timestampSeconds,
                measurement.standardDeviations
            );
        });
    }

    public void seedPoseFromVision() {
        if (limelight == null) {
            return;
        }
        final Pose2d currentPose = drivetrain.getState().Pose;
        limelight.getMegaTag1Measurement().ifPresent(measurement -> {
            final double jump = currentPose.getTranslation()
                .getDistance(measurement.poseEstimate.pose.getTranslation());
            if (jump < kMaxVisionJumpTeleopMeters || currentPose.getTranslation().getNorm() < 0.01 || !visionSeeded) {
                drivetrain.resetPose(measurement.poseEstimate.pose);
                visionSeeded = true;
                SmartDashboard.putBoolean("VisionSeeded", true);
            }
        });
    }
}
