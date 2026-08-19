package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Millimeters;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Value;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HoodSubsys extends SubsystemBase {
    private static final int kLeftServoPWM = 0;
    private static final int kRightServoPWM = 1;

    private static final Distance kServoLength = Millimeters.of(100);
    private static final LinearVelocity kMaxServoSpeed = Millimeters.of(20).per(Second);
    private static final double kMinPosition = 0.01;
    private static final double kMaxPosition = 0.77;
    private static final double kPositionTolerance = 0.01;

    private final Servo leftServo;
    private final Servo rightServo;
    private final Mechanism2d hoodMech = new Mechanism2d(60, 60);
    private final MechanismLigament2d hoodArm;

    private double currentPosition = 0.0;
    private double targetPosition = 0.0;
    private double smoothedPosition = 0.0;
    private Time lastUpdateTime = Seconds.of(0);

    // EMA smoothing factor: 0.0 = no change, 1.0 = no smoothing
    // 0.15 blends ~15% new value per cycle — smooth but still responsive
    private static final double kSmoothingAlpha = 1.0;

    public HoodSubsys() {
        leftServo = new Servo(kLeftServoPWM);
        rightServo = new Servo(kRightServoPWM);
        leftServo.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);
        rightServo.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);
        hoodArm = hoodMech.getRoot("Pivot", 8, 8)
            .append(new MechanismLigament2d("Hood", 40, 12, 8, new Color8Bit(Color.kOrange)));
        SmartDashboard.putData("Hood", hoodMech);
        SmartDashboard.putData(this);
        setPosition(frc.robot.Constants.ShooterConstants.kDefaultHoodPosition);
        currentPosition = targetPosition;
        smoothedPosition = targetPosition;
    }

    /** Expects a position between 0.0 and 1.0. Applies EMA smoothing to reduce jitter. */
    public void setPosition(double position) {
        final double clampedPosition = MathUtil.clamp(position, kMinPosition, kMaxPosition);
        smoothedPosition += kSmoothingAlpha * (clampedPosition - smoothedPosition);
        leftServo.set(smoothedPosition);
        rightServo.set(smoothedPosition);
        targetPosition = smoothedPosition;
        updateHoodGraphic();
    }

    public double getPosition() {
        return targetPosition;
    }

    /** Approximate launch elevation for HUD/sim (0.01 ≈ 12°, 0.77 ≈ 70°). */
    public double getAngleDegrees() {
        final double span = kMaxPosition - kMinPosition;
        final double t = MathUtil.clamp((targetPosition - kMinPosition) / span, 0.0, 1.0);
        return 12.0 + t * 58.0;
    }

    private void updateHoodGraphic() {
        hoodArm.setAngle(getAngleDegrees());
    }

    /** Expects a position between 0.0 and 1.0 */
    public Command positionCommand(double position) {
        return runOnce(() -> setPosition(position))
            .andThen(Commands.waitUntil(this::isPositionWithinTolerance));
    }

    public boolean isPositionWithinTolerance() {
        return MathUtil.isNear(targetPosition, currentPosition, kPositionTolerance);
    }

    private void updateCurrentPosition() {
        final Time currentTime = Seconds.of(Timer.getFPGATimestamp());
        final Time elapsedTime = currentTime.minus(lastUpdateTime);
        lastUpdateTime = currentTime;

        if (isPositionWithinTolerance()) {
            currentPosition = targetPosition;
            return;
        }

        final Distance maxDistanceTraveled = kMaxServoSpeed.times(elapsedTime);
        final double maxPercentageTraveled = maxDistanceTraveled.div(kServoLength).in(Value);
        currentPosition = targetPosition > currentPosition
            ? Math.min(targetPosition, currentPosition + maxPercentageTraveled)
            : Math.max(targetPosition, currentPosition - maxPercentageTraveled);
    }

    @Override
    public void periodic() {
        updateCurrentPosition();
        updateHoodGraphic();
        SmartDashboard.putBoolean("Hood At Position", isPositionWithinTolerance());
        SmartDashboard.putNumber("Hood Position", targetPosition);
        SmartDashboard.putNumber("Hood Angle (deg)", getAngleDegrees());
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addStringProperty("Command", () -> getCurrentCommand() != null ? getCurrentCommand().getName() : "null", null);
        builder.addDoubleProperty("Current Position", () -> currentPosition, null);
        builder.addDoubleProperty("Target Position", () -> targetPosition, value -> setPosition(value));
    }
}
