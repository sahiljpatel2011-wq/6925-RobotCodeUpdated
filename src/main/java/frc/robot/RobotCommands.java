package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static frc.robot.Constants.ShooterConstants.*;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
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
    private static double lastGoodAimTx = 0.0;
    private static double lastGoodAimInches = 75.125;
    private static double lastGoodAimTime = -1.0;
    private static double lastGoodPassTx = 0.0;
    private static double lastGoodPassTime = -1.0;

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
        final double[] state = {Double.NaN, 0};
        return Commands.runEnd(
            () -> {
                feederSubsys.setSpeed(FeederSpeed.FEED_FAST);
                intakeSubsys.setSpeed(IntakeSpeed.INTAKE_FAST);
                tickOscillate(state, oscillationMotorRotations, period);
            },
            () -> {
                feederSubsys.setSpeed(FeederSpeed.OFF);
                intakeSubsys.setSpeed(IntakeSpeed.OFF);
                restoreOscillate(state);
                // Keep shot RPM in auto so path-event "shoot" does not idle before autoShoot.
                if (!isAimHeld() && !DriverStation.isAutonomous()) {
                    shooterSubsys.returnToIdle();
                }
            },
            feederSubsys, intakeSubsys
        );
    }

    public static Command autoShoot(double seconds) {
        final double oscillationMotorRotations = (60.0 / 360.0) * 8.0;
        final double period = 0.8;
        final double[] state = {Double.NaN, 0};
        return Commands.runEnd(
            () -> {
                feederSubsys.setSpeed(FeederSpeed.FEED_FAST);
                intakeSubsys.setSpeed(IntakeSpeed.INTAKE_FAST);
                tickOscillate(state, oscillationMotorRotations, period);
            },
            () -> {
                feederSubsys.setSpeed(FeederSpeed.OFF);
                intakeSubsys.setSpeed(IntakeSpeed.OFF);
                restoreOscillate(state, -14.5);
                shooterSubsys.returnToIdle();
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
                final var hubTag = limelightSubsys.hubAimTag();
                Distance distance;
                double tx = 0.0;
                int tagID = 0;
                final double now = Timer.getFPGATimestamp();
                if (hubTag.isPresent()) {
                    tagID = hubTag.get().id;
                    final double rawTx = hubTag.get().txDegrees;
                    final double ty = hubTag.get().tyDegrees;
                    final var cameraToTag = HubAimMath.cameraToTagInches(ty);
                    if (cameraToTag.isPresent()) {
                        distance = Inches.of(movingShotInches(
                            cameraToTag.getAsDouble() + kHubCenterOffsetInches));
                        tx = HubAimMath.hubAimTxDegrees(tagID, rawTx, ty).orElse(rawTx)
                            + kAimOffsetDegrees
                            + movingLeadTxDegrees();
                    } else {
                        distance = getPredictedDistanceToTarget();
                        tx = rawTx + kAimOffsetDegrees + movingLeadTxDegrees();
                    }
                    lastGoodAimTx = tx;
                    lastGoodAimInches = distance.in(Inches);
                    lastGoodAimTime = now;
                } else if (HubAimMath.keepLastAim(now, lastGoodAimTime, HubAimMath.kAimHoldSeconds)) {
                    tx = lastGoodAimTx;
                    distance = Inches.of(lastGoodAimInches);
                } else {
                    distance = getPredictedDistanceToTarget();
                }
                lastTx = tx;
                lastTxTime = now;
                final double rotationRate = HubAimMath.aimAssistOmega(tx, kAimP);
                drivetrain.setControl(aimDrive
                    .withVelocityX(velocityX.getAsDouble())
                    .withVelocityY(velocityY.getAsDouble())
                    .withRotationalRate(rotationRate));

                final Shot shot = ShotTable.get(distance);
                shooterSubsys.setVelocityRPM(shot.shooterRPM);
                hoodSubsys.setPosition(shot.hoodPosition);
                SmartDashboard.putNumber("Tracked Tag ID", tagID);
                SmartDashboard.putNumber("Auto Distance (inches)", distance.in(Inches));
                SmartDashboard.putNumber("Corrected TX (deg)", tx);
                SmartDashboard.putBoolean("LL TV (code)", LimelightHelpers.getTV("limelight"));
                SmartDashboard.putNumber("LL TX (code)", LimelightHelpers.getTX("limelight"));
                SmartDashboard.putNumber("Aim Rotation Rate", rotationRate);
                SmartDashboard.putNumber("Target RPM", shot.shooterRPM);
                SmartDashboard.putNumber("Target Hood Position", shot.hoodPosition);
                SmartDashboard.putBoolean("On Target", Math.abs(tx) < HubAimMath.kOnTargetDegrees);
                SmartDashboard.putBoolean("Aim Used Hub Tag", HubAimMath.isHubTag(tagID));
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
                final boolean isTrenchTag = tagID == 7 || tagID == 12 || tagID == 23 || tagID == 28;
                double tx = 0.0;
                final double now = Timer.getFPGATimestamp();
                if (LimelightHelpers.getTV("limelight") && isTrenchTag) {
                    final double rawTx = LimelightHelpers.getTX("limelight");
                    final double offset = (tagID == 12 || tagID == 28)
                        ? -kPassAimOffsetDegrees
                        : kPassAimOffsetDegrees;
                    tx = rawTx + offset;
                    lastGoodPassTx = tx;
                    lastGoodPassTime = now;
                } else if (HubAimMath.keepLastAim(now, lastGoodPassTime, HubAimMath.kAimHoldSeconds)) {
                    tx = lastGoodPassTx;
                }
                final double rotationRate = HubAimMath.aimAssistOmega(tx, kAimP);

                drivetrain.setControl(passDrive
                    .withVelocityX(velocityX.getAsDouble())
                    .withVelocityY(velocityY.getAsDouble())
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

    public static Command autoshootWhileAimed() {
        final double oscillationMotorRotations = (60.0 / 360.0) * 8.0;
        final double period = 0.8;
        final double[] state = {Double.NaN, 0};
        return Commands.runEnd(
            () -> {
                final boolean onTarget = HubAimMath.isOnTarget(lastTx)
                    && HubAimMath.keepLastAim(
                        Timer.getFPGATimestamp(), lastTxTime, HubAimMath.kAimHoldSeconds);
                if (HubAimMath.autoFeed(
                    shooterSubsys.isReadyToFeed(), onTarget, feederSubsys.isFeeding())) {
                    feederSubsys.setSpeed(FeederSpeed.FEED_FAST);
                    intakeSubsys.setSpeed(IntakeSpeed.INTAKE_FAST);
                    tickOscillate(state, oscillationMotorRotations, period);
                } else {
                    feederSubsys.setSpeed(FeederSpeed.OFF);
                    intakeSubsys.setSpeed(IntakeSpeed.OFF);
                }
            },
            () -> {
                feederSubsys.setSpeed(FeederSpeed.OFF);
                intakeSubsys.setSpeed(IntakeSpeed.OFF);
                restoreOscillate(state);
            },
            feederSubsys, intakeSubsys
        );
    }

    public static Command autoshootFeed(GenericHID rumbleHid) {
        final double oscillationMotorRotations = (60.0 / 360.0) * 8.0;
        final double period = 0.8;
        final double[] state = {Double.NaN, 0};
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
                    tickOscillate(state, oscillationMotorRotations, period);
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
                restoreOscillate(state);
                if (!isAimHeld() && !DriverStation.isAutonomous()) {
                    shooterSubsys.returnToIdle();
                }
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
        final ChassisSpeeds speeds = fieldSpeeds();
        final Translation2d futurePosition = currentPose.getTranslation().plus(
            new Translation2d(
                speeds.vxMetersPerSecond * kLookAheadSeconds,
                speeds.vyMetersPerSecond * kLookAheadSeconds
            )
        );
        return Meters.of(futurePosition.getDistance(Landmarks.targetPosition()));
    }

    private static ChassisSpeeds fieldSpeeds() {
        final Pose2d pose = drivetrain.getState().Pose;
        return ChassisSpeeds.fromRobotRelativeSpeeds(
            drivetrain.getState().Speeds, pose.getRotation());
    }

    /** Limelight hub range, then the existing 0.25 s look-ahead while translating. */
    private static double movingShotInches(double cameraHubInches) {
        if (!Landmarks.isAllianceKnown()) {
            return cameraHubInches;
        }
        final Pose2d pose = drivetrain.getState().Pose;
        final ChassisSpeeds speeds = fieldSpeeds();
        final Translation2d hub = Landmarks.targetPosition();
        return HubAimMath.movingShotInches(
            cameraHubInches,
            pose.getX(),
            pose.getY(),
            hub.getX(),
            hub.getY(),
            speeds.vxMetersPerSecond,
            speeds.vyMetersPerSecond,
            kLookAheadSeconds);
    }

    private static double movingLeadTxDegrees() {
        if (!Landmarks.isAllianceKnown()) {
            return 0.0;
        }
        final Pose2d pose = drivetrain.getState().Pose;
        final ChassisSpeeds speeds = fieldSpeeds();
        final Translation2d hub = Landmarks.targetPosition();
        return HubAimMath.movingLeadTxDegrees(
            pose.getX(),
            pose.getY(),
            pose.getRotation().getRadians(),
            hub.getX(),
            hub.getY(),
            speeds.vxMetersPerSecond,
            speeds.vyMetersPerSecond,
            kLookAheadSeconds);
    }

    public static Command adjustedWindUp() {
        return Commands.run(() -> applyTableShot(getPredictedDistanceToTarget()), shooterSubsys, hoodSubsys);
    }

    public static Command adjustedShootWhileMoving() {
        return Commands.sequence(
            Commands.run(() -> applyTableShot(getPredictedDistanceToTarget()), shooterSubsys, hoodSubsys)
            .until(shooterSubsys::isVelocityWithinTolerance)
            .withTimeout(2.0),
            Commands.run(() -> {
                applyTableShot(getPredictedDistanceToTarget());
                feederSubsys.setSpeed(FeederSpeed.FEED_FAST);
            }, shooterSubsys, hoodSubsys, feederSubsys)
        ).finallyDo(() -> feederSubsys.setSpeed(FeederSpeed.OFF));
    }

    public static Command adjustedWindUpOnce() {
        return Commands.runOnce(() -> applyTableShot(getDistanceToTarget()), shooterSubsys, hoodSubsys)
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
        SmartDashboard.putBoolean("On Target", Timer.getFPGATimestamp() - lastTxTime < HubAimMath.kAimHoldSeconds && Math.abs(lastTx) < HubAimMath.kOnTargetDegrees);
        SmartDashboard.putNumber("Hood Position", hoodSubsys.getPosition());
        SmartDashboard.putNumber("Hood Angle (deg)", hoodSubsys.getAngleDegrees());
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

    private static void applyTableShot(Distance distance) {
        final Shot shot = ShotTable.get(distance);
        shooterSubsys.setVelocityRPM(shot.shooterRPM);
        hoodSubsys.setPosition(shot.hoodPosition);
        SmartDashboard.putNumber("Distance to Target (inches)", distance.in(Inches));
        SmartDashboard.putNumber("Target RPM", shot.shooterRPM);
        SmartDashboard.putNumber("Target Hood Position", shot.hoodPosition);
    }

    private static void tickOscillate(double[] state, double amplitudeRot, double period) {
        if (Double.isNaN(state[0])) {
            state[0] = Timer.getFPGATimestamp();
            state[1] = intakeSubsys.getRotatorPosition();
        }
        final double elapsed = Timer.getFPGATimestamp() - state[0];
        final boolean goUp = ((int) (elapsed / (period / 2.0)) % 2 == 0);
        intakeSubsys.setRotatorOscillate(goUp ? state[1] + amplitudeRot : state[1]);
    }

    private static void restoreOscillate(double[] state) {
        restoreOscillate(state, Double.NaN);
    }

    private static void restoreOscillate(double[] state, double fallback) {
        if (!Double.isNaN(state[0])) {
            intakeSubsys.setRotatorTarget(state[1]);
        } else if (!Double.isNaN(fallback)) {
            intakeSubsys.setRotatorTarget(fallback);
        }
        state[0] = Double.NaN;
    }

    private static boolean isAimHeld() {
        return shooterSubsys.getCurrentCommand() != null
            && shooterSubsys.getCurrentCommand() != shooterSubsys.getDefaultCommand();
    }
}
