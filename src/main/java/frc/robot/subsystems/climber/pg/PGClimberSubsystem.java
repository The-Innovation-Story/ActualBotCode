// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber.pg;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
// import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class PGClimberSubsystem extends SubsystemBase {
  private final WPI_TalonSRX pg_Andar, pg_Bahar;

  /** Creates a new PGClimberSubsystem. */
  public PGClimberSubsystem() {
    this.pg_Andar = new WPI_TalonSRX(4);
    this.pg_Bahar = new WPI_TalonSRX(6);
    this.pg_Andar.setSelectedSensorPosition(0.0);
    this.pg_Bahar.setSelectedSensorPosition(0.0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("PG Inner", this.pg_Andar.getSelectedSensorPosition(0));
    SmartDashboard.putNumber("PG Inner CY", this.pg_Andar.getSelectedSensorPosition());
    SmartDashboard.putNumber("PG Andar L", this.pg_Andar.get());
    SmartDashboard.putNumber("PG Bahar R", this.pg_Andar.get());
    // This method will be called once per scheduler run
  }

  public double getInnerPGPosition() {
    return this.pg_Andar.getSelectedSensorPosition();
  }

  public void setInnerPGBasePosition() {
    this.pg_Andar.setSelectedSensorPosition(0.0);
  }

  public double getOuterPGPosition() {
    return this.pg_Bahar.getSelectedSensorPosition();
  }

  public void setOuterPGBasePosition() {
    this.pg_Bahar.setSelectedSensorPosition(0.0);
  }

  public double getAndarWalaPosition() {
    return this.pg_Bahar.getSelectedSensorPosition();
  }

  public void setPGInnerSpeed(double pg) {
    this.pg_Andar.set(TalonSRXControlMode.PercentOutput, pg * ClimberConstants.speedMultiplier);
  }

  public void setPGInnerPIDSpeed(double pg) {
    this.pg_Andar.set(TalonSRXControlMode.PercentOutput, pg);
  }

  public void setPGOuterPIDSpeed(double pg) {
    this.pg_Bahar.set(TalonSRXControlMode.PercentOutput, pg);
  }

  public void setPGOuterSpeed(double pg) {
    this.pg_Bahar.set(TalonSRXControlMode.PercentOutput, pg * ClimberConstants.speedMultiplier);
  }
}
