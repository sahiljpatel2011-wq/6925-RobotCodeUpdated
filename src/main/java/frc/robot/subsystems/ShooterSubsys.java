// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.CTREConfigs;
import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class ShooterSubsys extends SubsystemBase {
  private final TalonFX fuelShoot = new TalonFX(8, "CANivore");
  private final TalonFX fuelShoot0 = new TalonFX(9, "CANivore");
  private final TalonFX fuelShoot1 = new TalonFX(10, "CANivore");

  public static final double kIdleRPM = ShooterConstants.kIdleRPM;
  private static final double kVelocityToleranceRPM = 300.0;
  private static final double kFeedReadyMarginRPM = 400.0;

  private double holdRPM = kIdleRPM;
  private double targetRPM = kIdleRPM;

  public ShooterSubsys() {
    fuelShoot.getConfigurator().apply(CTREConfigs.SHOOTER_CONFIG);
    fuelShoot0.getConfigurator().apply(CTREConfigs.SHOOTER_CONFIG_9);
    fuelShoot1.getConfigurator().apply(CTREConfigs.SHOOTER_CONFIG_10);
    setDefaultCommand(Commands.run(() -> setVelocityRPM(holdRPM), this));
  }

  private final VelocityVoltage m_velocityRequest = new VelocityVoltage(0);
  private final VelocityVoltage m_velocityRequest0 = new VelocityVoltage(0);
  private final VelocityVoltage m_velocityRequest1 = new VelocityVoltage(0);

  public void setVelocityRPM(double rpm) {
    targetRPM = rpm;
    holdRPM = rpm;
    double rps = rpm / 60.0;
    fuelShoot.setControl(m_velocityRequest.withVelocity(rps));
    fuelShoot0.setControl(m_velocityRequest0.withVelocity(rps));
    fuelShoot1.setControl(m_velocityRequest1.withVelocity(rps));
  }

  public void returnToIdle() {
    setVelocityRPM(kIdleRPM);
  }

  /** Spins only the right (leader) motor at the given RPM. Others are stopped. */
  public void setRightMotorOnly(double rpm) {
    fuelShoot0.stopMotor();
    fuelShoot1.stopMotor();
    double rps = rpm / 60.0;
    fuelShoot.setControl(m_velocityRequest.withVelocity(rps));
  }

  /** Returns flywheels to idle 3000 instead of coasting to zero. */
  public void stopShooter() {
    returnToIdle();
  }

  public Command setVelocityRPMCommand(double rpm) {
    return Commands.runOnce(() -> setVelocityRPM(rpm), this);
  }

  public double getVelocityRPM() {
    return getVelocityRPM8();
  }

  public double getVelocityRPM8() {
    return fuelShoot.getVelocity().getValueAsDouble() * 60.0;
  }

  public double getVelocityRPM9() {
    return fuelShoot0.getVelocity().getValueAsDouble() * 60.0;
  }

  public double getVelocityRPM10() {
    return fuelShoot1.getVelocity().getValueAsDouble() * 60.0;
  }

  public double getTargetRPM() {
    return targetRPM;
  }

  public double getHoldRPM() {
    return holdRPM;
  }

  public boolean isIdleHold() {
    return holdRPM <= kIdleRPM + 50.0;
  }

  public boolean isSpooling() {
    return holdRPM > kIdleRPM + 50.0 && !isVelocityWithinTolerance();
  }

  public boolean isVelocityWithinTolerance() {
    if (isIdleHold() || targetRPM <= 0) {
      return false;
    }
    return columnInBand(getVelocityRPM8())
        && columnInBand(getVelocityRPM9())
        && columnInBand(getVelocityRPM10());
  }

  /** True if every column is at least target-400 RPM (AutoshootFeed gate). */
  public boolean isReadyToFeed() {
    if (isIdleHold() || targetRPM <= 0) {
      return false;
    }
    final double minRpm = targetRPM - kFeedReadyMarginRPM;
    return getVelocityRPM8() >= minRpm
        && getVelocityRPM9() >= minRpm
        && getVelocityRPM10() >= minRpm
        && isVelocityWithinTolerance();
  }

  private boolean columnInBand(double rpm) {
    return Math.abs(rpm - targetRPM) < kVelocityToleranceRPM;
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Shooter At Speed", isVelocityWithinTolerance());
    SmartDashboard.putNumber("Shooter RPM", getVelocityRPM8());
    SmartDashboard.putNumber("Shooter RPM Motor 9", getVelocityRPM9());
    SmartDashboard.putNumber("Shooter RPM Motor 10", getVelocityRPM10());
    SmartDashboard.putNumber("Shooter Target RPM", targetRPM);
  }
}
