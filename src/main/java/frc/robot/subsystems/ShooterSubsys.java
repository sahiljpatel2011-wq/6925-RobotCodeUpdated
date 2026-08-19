// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CTREConfigs;
import frc.robot.Constants.ShooterConstants;


public class ShooterSubsys extends SubsystemBase {
  private final TalonFX fuelShoot = new TalonFX(8, "CANivore");
  private final TalonFX fuelShoot0 = new TalonFX(9, "CANivore");
  private final TalonFX fuelShoot1 = new TalonFX(10, "CANivore");

  private final StatusSignal<AngularVelocity> vel8 = fuelShoot.getVelocity();
  private final StatusSignal<AngularVelocity> vel9 = fuelShoot0.getVelocity();
  private final StatusSignal<AngularVelocity> vel10 = fuelShoot1.getVelocity();

  public static final double kIdleRPM = ShooterConstants.kIdleRPM;
  private static final double kVelocityToleranceRPM = 300.0;
  private static final double kFeedReadyMarginRPM = 400.0;

  private double holdRPM = kIdleRPM;
  private double targetRPM = kIdleRPM;

  public ShooterSubsys() {
    fuelShoot.getConfigurator().apply(CTREConfigs.SHOOTER_CONFIG);
    fuelShoot0.getConfigurator().apply(CTREConfigs.SHOOTER_CONFIG_9);
    fuelShoot1.getConfigurator().apply(CTREConfigs.SHOOTER_CONFIG_10);
    BaseStatusSignal.setUpdateFrequencyForAll(50.0, vel8, vel9, vel10);
    fuelShoot.optimizeBusUtilization();
    fuelShoot0.optimizeBusUtilization();
    fuelShoot1.optimizeBusUtilization();
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
    if (RobotBase.isSimulation()) {
      fuelShoot.getSimState().setSupplyVoltage(12.0);
      fuelShoot0.getSimState().setSupplyVoltage(12.0);
      fuelShoot1.getSimState().setSupplyVoltage(12.0);
      fuelShoot.getSimState().setRotorVelocity(rps);
      fuelShoot0.getSimState().setRotorVelocity(rps);
      fuelShoot1.getSimState().setRotorVelocity(rps);
    }
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
    return vel8.getValueAsDouble() * 60.0;
  }

  public double getVelocityRPM9() {
    return vel9.getValueAsDouble() * 60.0;
  }

  public double getVelocityRPM10() {
    return vel10.getValueAsDouble() * 60.0;
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

  /**
   * Motor 8 must be in the ±300 RPM band. 9 and 10 must also be in band when
   * their CAN signal is fresh so a stale column cannot brick "at speed".
   */
  public boolean isVelocityWithinTolerance() {
    if (isIdleHold() || targetRPM <= 0) {
      return false;
    }
    if (!columnInBand(getVelocityRPM8())) {
      return false;
    }
    return columnOkIfFresh(vel9, getVelocityRPM9())
        && columnOkIfFresh(vel10, getVelocityRPM10());
  }

  /** True if every live column is at least target-400 RPM (AutoshootFeed gate). */
  public boolean isReadyToFeed() {
    if (isIdleHold() || targetRPM <= 0) {
      return false;
    }
    final double minRpm = targetRPM - kFeedReadyMarginRPM;
    if (getVelocityRPM8() < minRpm) {
      return false;
    }
    if (isFresh(vel9) && getVelocityRPM9() < minRpm) {
      return false;
    }
    if (isFresh(vel10) && getVelocityRPM10() < minRpm) {
      return false;
    }
    return isVelocityWithinTolerance();
  }

  private boolean columnInBand(double rpm) {
    return Math.abs(rpm - targetRPM) < kVelocityToleranceRPM;
  }

  private boolean columnOkIfFresh(StatusSignal<AngularVelocity> signal, double rpm) {
    return !isFresh(signal) || columnInBand(rpm);
  }

  private static boolean isFresh(StatusSignal<?> signal) {
    return signal.getStatus().isOK() && signal.getTimestamp().getLatency() < 0.25;
  }

  @Override
  public void periodic() {
    BaseStatusSignal.refreshAll(vel8, vel9, vel10);
    SmartDashboard.putBoolean("Shooter At Speed", isVelocityWithinTolerance());
    SmartDashboard.putNumber("Shooter RPM", getVelocityRPM8());
    SmartDashboard.putNumber("Shooter RPM Motor 9", getVelocityRPM9());
    SmartDashboard.putNumber("Shooter RPM Motor 10", getVelocityRPM10());
    SmartDashboard.putNumber("Shooter Target RPM", targetRPM);
  }
}
