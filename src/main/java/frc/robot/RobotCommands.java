package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static frc.robot.Constants.ShooterConstants.*;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.FeederSubsys;
import frc.robot.subsystems.FeederSubsys.FeederSpeed;
import frc.robot.subsystems.HoodSubsys;
import frc.robot.subsystems.IntakeSubsys;
import frc.robot.subsystems.IntakeSubsys.IntakeSpeed;
import frc.robot.subsystems.LimelightSubsys;
import frc.robot.subsystems.ShooterSubsys;
import frc.robot.vision.HubAimMath;
import frc.robot.vision.ShotTable;
import frc.robot.vision.ShotTable.Shot;
import frc.robot.LimelightHelpers.RawDetection;
import frc.robot.LimelightHelpers.RawFiducial;

public final class RobotCommands {
    private static ShooterSubsys shooterSubsys;
    private static FeederSubsys feederSubsys;
    private static HoodSubsys hoodSubsys;
    private static IntakeSubsys intakeSubsys;
    private static CommandSwerveDrivetrain drivetrain;
    private static LimelightSubsys limelightSubsys;

    private static double lastTx = 0.0;
    private static double lastTxTime = 0.0;
    private static String lastMode = "Idle";

    public static void init(
        ShooterSubsys shooter,
        FeederSubsys feeder,
        HoodSubsys hood,
        IntakeSubsys intake,
        CommandSwerveDrivetrain drive,
        LimelightSubsys limelight
    ) {
        RobotCommands.shooterSubsys = shooter;
        RobotCommands.feederSubsys = feeder;
        RobotCommands.hoodSubsys = hood;
        RobotCommands.intakeSubsys = intake;
        RobotCommands.drivetrain = drive;
        RobotCommands.limelightSubsys = limelight;
        ShotTable.publishNtDefaults();
        FeatureFlags.publishDefaults();
    }

    public static Command windUpOnce() {
        return Commands.runOnce(() -> {
            shooterSubsys.setVelocityRPM(kFixedShotRPM);
            hoodSubsys.setPosition(kDefaultHoodPosition);
        }, shooterSubsys, hoodSubsys);
    }

    public static Command autoWindUp() {
        return Commands.runOnce(() -> {
            shooterSubsys.setVelocityRPM(kFixedShotRPM);
            hoodSubsys.setPosition(kDefaultHoodPosition);
        }, shooterSubsys, hoodSubsys)
        .andThen(Commands.waitUntil(shooterSubsys::isVelocityWithinTolerance).withTimeout(2.0));
    }

    public static Command autoWindUpClose() {
        return Commands.runOnce(() -> {
            shooterSubsys.setVelocityRPM(kFixedShotRPM);
            hoodSubsys.setPosition(kCloseHoodPosition);
        }, shooterSubsys, hoodSubsys)
        .andThen(Commands.waitUntil(shooterSubsys::isVelocityWithinTolerance).withTimeout(2.0));
    }

    public static Command autoWindUpCloser() {
        return Commands.runOnce(() -> {
            shooterSubsys.setVelocityRPM(kFixedShotRPM);
            hoodSubsys.setPosition(kCloserHoodPosition);
        }, shooterSubsys, hoodSubsys)
        .andThen(Commands.waitUntil(shooterSubsys::isVelocityWithinTolerance).withTimeout(2.0));
    }

    public static Command windUp() {
        return holdWindUp(kFixedShotRPM, kDefaultHoodPosition);
    }

    public static Command windUpClose() {
        return holdWindUp(kFixedShotRPM, kCloseHoodPosition);
    }

    public static Command windUpCloser() {
        return holdWindUp(kFixedShotRPM, kCloserHoodPosition);
    }

    public static Command windUpPass() {
        return holdWindUp(kPassShotRPM, kPassHoodPosition);
    }

    public static Command windUp110() {
        return holdWindUp(kRPMAt110in, kHoodAt110in);
    }

    public static Command windUp75() {
        return holdWindUp(kRPMAt75in, kHoodAt75in);
    }

    public static Command windUpTest() {
        return holdWindUp(kFixedShotRPM, kTestHoodPosition);
    }

    private static Command holdWindUp(double rpm, double hood) {
        return Commands.runEnd(
            () -> {
                shooterSubsys.setVelocityRPM(rpm);
                hoodSubsys.setPosition(hood);
            },
            () -> {
                if (!feederSubsys.isFeeding()) {
                    shooterSubsys.returnToIdle();
                }
            },
            shooterSubsys, hoodSubsys
        );
    }

    public static Command windUpAndShoot() {
        return Commands.runEnd(
            () -> {
                shooterSubsys.setVelocityRPM(kFixedShotRPM);
                hoodSubsys.setPosition(kDefaultHoodPosition);
                feederSubsys.setSpeed(FeederSpeed.FEED_FAST);
            },
            () -> {
                shooterSubsys.returnToIdle();
                feederSubsys.setSpeed(FeederSpeed.OFF);
            },
            shooterSubsys, hoodSubsys, feederSubsys
        );
    }

    public static Command Shoot() {
        final double oscillationMotorRotations = (60.0 / 360.0) * 8.0;
        final double period = 0.8;
        final double[] state = {0, 0};
        return Commands.runEnd(
            () -> {
                feederSubsys.setSpeed(FeederSpeed.FEED_FAST);
                intakeSubsys.setSpeed(IntakeSpeed.INTAKE_FAST);
                if (state[0] == 0) {
                    state[0] = Timer.getFPGATimestamp();
                    state[1] = intakeSubsys.getRotatorPosition();
                }
                double elapsed = Timer.getFPGATimestamp() - state[0];
                boolean goUp = ((int)(elapsed / (period / 2.0)) % 2 == 0);
                double target = goUp ? state[1] + oscillationMotorRotations : state[1];
                intakeSubsys.setRotatorOscillate(target);
            },
            () -> {
                feederSubsys.setSpeed(FeederSpeed.OFF);
                intakeSubsys.setSpeed(IntakeSpeed.OFF);
                intakeSubsys.setRotatorTarget(state[1]);
                if (!isAimHeld()) {
                    shooterSubsys.returnToIdle();
                }
                state[0] = 0;
            },
            feederSubsys, intakeSubsys
        );
    }

    public static Command autoShoot(double seconds) {
        final double oscillationMotorRotations = (60.0 / 360.0) * 8.0;
        final double period = 0.8;
        final double[] state = {0, 0};
        return Commands.runEnd(
            () -> {
                feederSubsys.setSpeed(FeederSpeed.FEED_FAST);
                intakeSubsys.setSpeed(IntakeSpeed.INTAKE_FAST);
                if (state[0] == 0) {
                    state[0] = Timer.getFPGATimestamp();
                    state[1] = intakeSubsys.getRotatorPosition();
                }
                double elapsed = Timer.getFPGATimestamp() - state[0];
                boolean goUp = ((int)(elapsed / (period / 2.0)) % 2 == 0);
                double target = goUp ? state[1] + oscillationMotorRotations : state[1];
                intakeSubsys.setRotatorOscillate(target);
            },
            () -> {
                feederSubsys.setSpeed(FeederSpeed.OFF);
                intakeSubsys.setSpeed(IntakeSpeed.OFF);
                intakeSubsys.setRotatorTarget(state[1] != 0 ? state[1] : -14.5);
                shooterSubsys.returnToIdle();
                state[0] = 0;
            },
            feederSubsys, intakeSubsys
        ).withTimeout(seconds);
    }

    public static Command stopFeed() {
        return Commands.runOnce(() -> {
            feederSubsys.setSpeed(FeederSpeed.OFF);
            shooterSubsys.returnToIdle();
        }, feederSubsys);
    }

    public static Command intakeMid() {
        return intakeSubsys.setSpeedCommand(IntakeSpeed.INTAKE_MID);
    }

    public static Command intakeFast() {
        return intakeSubsys.setSpeedCommand(IntakeSpeed.INTAKE_FAST);
    }

    public static Command stopIntake() {
        return intakeSubsys.setSpeedCommand(IntakeSpeed.OFF);
    }

    public static Command reverseAll() {
        return Commands.runEnd(
            () -> {
                intakeSubsys.setSpeed(IntakeSpeed.REVERSE);
                feederSubsys.setSpeed(FeederSpeed.REVERSE);
            },
            () -> {
                intakeSubsys.setSpeed(IntakeSpeed.OFF);
                feederSubsys.setSpeed(FeederSpeed.OFF);
            },
            intakeSubsys, feederSubsys
        );
    }

    public static Command aimAndWindUp(DoubleSupplier velocityX, DoubleSupplier velocityY, double maxSpeed) {
        final SwerveRequest.FieldCentric aimDrive = new SwerveRequest.FieldCentric()
            .withDeadband(maxSpeed * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        return Commands.runEnd(() -> {
                final AimSnapshot aim = computeHubAim();
                final double rotationRate = pdRotation(-aim.tx);
                final double transScale = shooterSubsys.isSpooling() ? 0.75 : 1.0;
                drivetrain.setControl(aimDrive
                    .withVelocityX(velocityX.getAsDouble() * transScale)
                    .withVelocityY(velocityY.getAsDouble() * transScale)
                    .withRotationalRate(rotationRate));

                final Shot shot = ShotTable.get(aim.distance);
                shooterSubsys.setVelocityRPM(shot.shooterRPM);
                hoodSubsys.setPosition(shot.hoodPosition);
                publishAim(aim, shot, rotationRate);
            },
            () -> {
                if (!feederSubsys.isFeeding()) {
                    shooterSubsys.returnToIdle();
                }
            },
            drivetrain, shooterSubsys, hoodSubsys);
    }

    public static Command aimAndPassFullField(DoubleSupplier velocityX, DoubleSupplier velocityY, double maxSpeed) {
        final SwerveRequest.FieldCentric passDrive = new SwerveRequest.FieldCentric()
            .withDeadband(maxSpeed * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        return Commands.runEnd(() -> {
                shooterSubsys.setVelocityRPM(kFullFieldPassShotRPM);
                hoodSubsys.setPosition(kFullFieldPassHoodPosition);

                final int tagID = (int) LimelightHelpers.getFiducialID("limelight");
                double rotationRate = 0.0;
                if (LimelightHelpers.getTV("limelight") && HubAimMath.isTrenchTag(tagID)) {
                    final double rawTx = LimelightHelpers.getTX("limelight");
                    final double correctedTx = rawTx + HubAimMath.passAimOffsetDegrees(tagID);
                    rotationRate = pdRotation(-correctedTx);
                }

                final double transScale = shooterSubsys.isSpooling() ? 0.75 : 1.0;
                drivetrain.setControl(passDrive
                    .withVelocityX(velocityX.getAsDouble() * transScale)
                    .withVelocityY(velocityY.getAsDouble() * transScale)
                    .withRotationalRate(rotationRate));
            },
            () -> {
                if (!feederSubsys.isFeeding()) {
                    shooterSubsys.returnToIdle();
                }
            },
            drivetrain, shooterSubsys, hoodSubsys
        );
    }

    public static Command fuelAssist(DoubleSupplier velocityX, DoubleSupplier velocityY, double maxSpeed) {
        final SwerveRequest.FieldCentric assistDrive = new SwerveRequest.FieldCentric()
            .withDeadband(maxSpeed * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        return Commands.runEnd(() -> {
                if (!FeatureFlags.fuelAssist()) {
                    limelightSubsys.setPipeline(0);
                    drivetrain.setControl(assistDrive
                        .withVelocityX(velocityX.getAsDouble())
                        .withVelocityY(velocityY.getAsDouble())
                        .withRotationalRate(0));
                    return;
                }
                limelightSubsys.setPipeline(1);
                double bestTa = -1.0;
                double tx = 0.0;
                for (RawDetection detection : LimelightHelpers.getRawDetections("limelight")) {
                    if (detection.ta > bestTa) {
                        bestTa = detection.ta;
                        tx = detection.txnc;
                    }
                }
                drivetrain.setControl(assistDrive
                    .withVelocityX(velocityX.getAsDouble())
                    .withVelocityY(velocityY.getAsDouble())
                    .withRotationalRate(bestTa > 0 ? pdRotation(-tx) : 0));
            },
            () -> limelightSubsys.setPipeline(0),
            drivetrain
        );
    }

    public static Command autoshootFeed(GenericHID rumbleHid) {
        final double oscillationMotorRotations = (60.0 / 360.0) * 8.0;
        final double period = 0.8;
        final double[] state = {0, 0};
        return Commands.runEnd(
            () -> {
                if (!FeatureFlags.autoshootFeed()) {
                    rumbleHid.setRumble(GenericHID.RumbleType.kBothRumble, 0);
                    feederSubsys.setSpeed(FeederSpeed.OFF);
                    return;
                }
                if (shooterSubsys.isReadyToFeed()) {
                    rumbleHid.setRumble(GenericHID.RumbleType.kBothRumble, 0.4);
                    feederSubsys.setSpeed(FeederSpeed.FEED_FAST);
                    intakeSubsys.setSpeed(IntakeSpeed.INTAKE_FAST);
                    if (state[0] == 0) {
                        state[0] = Timer.getFPGATimestamp();
                        state[1] = intakeSubsys.getRotatorPosition();
                    }
                    double elapsed = Timer.getFPGATimestamp() - state[0];
                    boolean goUp = ((int)(elapsed / (period / 2.0)) % 2 == 0);
                    intakeSubsys.setRotatorOscillate(goUp ? state[1] + oscillationMotorRotations : state[1]);
                } else {
                    rumbleHid.setRumble(GenericHID.RumbleType.kBothRumble, 0);
                    feederSubsys.setSpeed(FeederSpeed.OFF);
                    intakeSubsys.setSpeed(IntakeSpeed.OFF);
                }
            },
            () -> {
                rumbleHid.setRumble(GenericHID.RumbleType.kBothRumble, 0);
                feederSubsys.setSpeed(FeederSpeed.OFF);
                intakeSubsys.setSpeed(IntakeSpeed.OFF);
                intakeSubsys.setRotatorTarget(state[1]);
                if (!isAimHeld()) {
                    shooterSubsys.returnToIdle();
                }
                state[0] = 0;
            },
            feederSubsys, intakeSubsys
        );
    }

    private static Distance getDistanceToTarget() {
        if (!Landmarks.isAllianceKnown()) {
            return Inches.of(75.125);
        }
        final Translation2d robotPosition = drivetrain.getState().Pose.getTranslation();
        final Translation2d targetPosition = Landmarks.targetPosition();
        return Meters.of(robotPosition.getDistance(targetPosition));
    }

    private static Distance getPredictedDistanceToTarget() {
        if (!Landmarks.isAllianceKnown()) {
            return Inches.of(75.125);
        }
        final Pose2d currentPose = drivetrain.getState().Pose;
        final ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
            drivetrain.getState().Speeds, currentPose.getRotation());
        final Translation2d futurePosition = currentPose.getTranslation().plus(
            new Translation2d(
                fieldSpeeds.vxMetersPerSecond * kLookAheadSeconds,
                fieldSpeeds.vyMetersPerSecond * kLookAheadSeconds
            )
        );
        final Translation2d targetPosition = Landmarks.targetPosition();
        return Meters.of(futurePosition.getDistance(targetPosition));
    }

    public static Command adjustedWindUp() {
        return Commands.run(() -> {
            if (!Landmarks.isAllianceKnown()) {
                return;
            }
            final Distance distance = getPredictedDistanceToTarget();
            final Shot shot = ShotTable.get(distance);
            shooterSubsys.setVelocityRPM(shot.shooterRPM);
            hoodSubsys.setPosition(shot.hoodPosition);
            SmartDashboard.putNumber("Distance to Target (inches)", distance.in(Inches));
            SmartDashboard.putNumber("Target RPM", shot.shooterRPM);
            SmartDashboard.putNumber("Target Hood Position", shot.hoodPosition);
        }, shooterSubsys, hoodSubsys);
    }

    public static Command adjustedShootWhileMoving() {
        return Commands.sequence(
            Commands.run(() -> {
                final Distance distance = getPredictedDistanceToTarget();
                final Shot shot = ShotTable.get(distance);
                shooterSubsys.setVelocityRPM(shot.shooterRPM);
                hoodSubsys.setPosition(shot.hoodPosition);
            }, shooterSubsys, hoodSubsys)
            .until(shooterSubsys::isVelocityWithinTolerance)
            .withTimeout(2.0),
            Commands.run(() -> {
                final Distance distance = getPredictedDistanceToTarget();
                final Shot shot = ShotTable.get(distance);
                shooterSubsys.setVelocityRPM(shot.shooterRPM);
                hoodSubsys.setPosition(shot.hoodPosition);
                feederSubsys.setSpeed(FeederSpeed.FEED_FAST);
            }, shooterSubsys, hoodSubsys, feederSubsys)
        );
    }

    public static Command adjustedWindUpOnce() {
        return Commands.runOnce(() -> {
            if (!Landmarks.isAllianceKnown()) {
                return;
            }
            final Distance distance = getDistanceToTarget();
            final Shot shot = ShotTable.get(distance);
            shooterSubsys.setVelocityRPM(shot.shooterRPM);
            hoodSubsys.setPosition(shot.hoodPosition);
        }, shooterSubsys, hoodSubsys)
        .andThen(Commands.waitUntil(shooterSubsys::isVelocityWithinTolerance).withTimeout(2.0));
    }

    public static Command autoTuneExposure() {
        final double[] exposure = {10.0};
        final double[] tagSeenSince = {-1.0};
        final double[] lastStep = {0.0};
        final double kStep = 50.0;
        final double kMaxExposure = 10000.0;
        final double kStableTime = 0.35;
        final double kStepPeriod = 0.10;

        return Commands.run(() -> {
            LimelightHelpers.setLimelightNTDoubleArray("limelight", "sensor_set",
                new double[]{0, exposure[0], 1, 20});
            SmartDashboard.putNumber("LL Auto-Tune Exposure (us)", exposure[0]);

            if (LimelightHelpers.getTV("limelight")) {
                if (tagSeenSince[0] < 0) {
                    tagSeenSince[0] = Timer.getFPGATimestamp();
                }
            } else {
                tagSeenSince[0] = -1.0;
            }

            final double now = Timer.getFPGATimestamp();
            if (now - lastStep[0] >= kStepPeriod
                && (tagSeenSince[0] < 0 || now - tagSeenSince[0] < kStableTime)) {
                exposure[0] = Math.min(exposure[0] + kStep, kMaxExposure);
                lastStep[0] = now;
            }
        }).until(() ->
            tagSeenSince[0] > 0 && Timer.getFPGATimestamp() - tagSeenSince[0] >= kStableTime
        ).finallyDo(() ->
            SmartDashboard.putNumber("LL Tuned Exposure (us)", exposure[0])
        );
    }

    public static void updateHud() {
        SmartDashboard.putNumber("Battery Voltage", RobotController.getBatteryVoltage());
        SmartDashboard.putNumber("Shooter RPM 8", shooterSubsys.getVelocityRPM8());
        SmartDashboard.putNumber("Shooter RPM 9", shooterSubsys.getVelocityRPM9());
        SmartDashboard.putNumber("Shooter RPM 10", shooterSubsys.getVelocityRPM10());
        SmartDashboard.putNumber("LL Pipeline", limelightSubsys.getPipelineIndex());
        SmartDashboard.putString("Shooter Mode", shooterMode());
        SmartDashboard.putBoolean("On Target", Math.abs(lastTx) < 2.0);
    }

    private static String shooterMode() {
        if (feederSubsys.isFeeding()) {
            lastMode = "Feeding";
        } else if (shooterSubsys.getHoldRPM() >= kFullFieldPassShotRPM - 50) {
            lastMode = "Passing";
        } else if (shooterSubsys.isVelocityWithinTolerance()) {
            lastMode = "Ready";
        } else if (shooterSubsys.isSpooling()) {
            lastMode = "Spooling";
        } else if (intakeSubsys.getRotatorPosition() < -5.0) {
            lastMode = "Intaking";
        } else {
            lastMode = "Idle";
        }
        return lastMode;
    }

    private static boolean isAimHeld() {
        return shooterSubsys.getCurrentCommand() != null
            && shooterSubsys.getCurrentCommand() != shooterSubsys.getDefaultCommand();
    }

    private static double pdRotation(double txNegated) {
        final double tx = -txNegated;
        final double now = Timer.getFPGATimestamp();
        final double dt = now - lastTxTime;
        final double dTx = dt > 1e-3 ? (tx - lastTx) / dt : 0.0;
        lastTx = tx;
        lastTxTime = now;
        return -tx * kAimP - dTx * kAimD;
    }

    private static AimSnapshot computeHubAim() {
        final Optional<RawFiducial> hubTag = limelightSubsys.bestHubFiducial();
        if (hubTag.isPresent()) {
            final RawFiducial tag = hubTag.get();
            final var aimTx = HubAimMath.hubAimTxDegrees(tag.id, tag.txnc, tag.tync);
            final var range = HubAimMath.hubRangeInches(tag.id, tag.txnc, tag.tync);
            if (aimTx.isPresent() && range.isPresent()) {
                return new AimSnapshot(tag.id, aimTx.getAsDouble() + kAimOffsetDegrees, Inches.of(range.getAsDouble()), true);
            }
        }
        final Pose2d pose = drivetrain.getState().Pose;
        final Optional<Translation2d> hub = Landmarks.targetPositionOptional();
        if (hub.isEmpty()) {
            return new AimSnapshot(0, 0, Inches.of(75.125), false);
        }
        final double tx = HubAimMath.poseAimTxDegrees(
            pose.getX(), pose.getY(), pose.getRotation().getRadians(),
            hub.get().getX(), hub.get().getY());
        final Distance distance = getPredictedDistanceToTarget();
        return new AimSnapshot(0, tx + kAimOffsetDegrees, distance, false);
    }

    private static void publishAim(AimSnapshot aim, Shot shot, double rotationRate) {
        SmartDashboard.putNumber("Tracked Tag ID", aim.tagId);
        SmartDashboard.putNumber("Auto Distance (inches)", aim.distance.in(Inches));
        SmartDashboard.putNumber("Corrected TX (deg)", aim.tx);
        SmartDashboard.putBoolean("LL TV (code)", LimelightHelpers.getTV("limelight"));
        SmartDashboard.putNumber("LL TX (code)", LimelightHelpers.getTX("limelight"));
        SmartDashboard.putNumber("Aim Rotation Rate", rotationRate);
        SmartDashboard.putNumber("Target RPM", shot.shooterRPM);
        SmartDashboard.putNumber("Target Hood Position", shot.hoodPosition);
        SmartDashboard.putBoolean("On Target", Math.abs(aim.tx) < 2.0);
        SmartDashboard.putBoolean("Aim Used Hub Tag", aim.fromTag);
    }

    private static final class AimSnapshot {
        final int tagId;
        final double tx;
        final Distance distance;
        final boolean fromTag;

        AimSnapshot(int tagId, double tx, Distance distance, boolean fromTag) {
            this.tagId = tagId;
            this.tx = tx;
            this.distance = distance;
            this.fromTag = fromTag;
        }
    }
}
