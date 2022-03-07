// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber.pg;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.ClimberConstants.PGConstants;

public class PGClimberSubsystem extends SubsystemBase {
  private final WPI_TalonSRX inner_pg, outer_pg;
  /** Creates a new PGClimberSubsystem. */
  public PGClimberSubsystem() {
    this.inner_pg = new WPI_TalonSRX(PGConstants.innerPGConstants);
    this.outer_pg = new WPI_TalonSRX(PGConstants.outerPGConstants);
    this.inner_pg.setSelectedSensorPosition(0.0);
    this.outer_pg.setSelectedSensorPosition(0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("PG Andar L", this.inner_pg.get());
    SmartDashboard.putNumber("PG Bahar R", this.outer_pg.get());
  }

  public double getInnerPGPosition() {
    return this.inner_pg.getSelectedSensorPosition();
  }

  public void setInnerPGBasePosition() {
    this.inner_pg.setSelectedSensorPosition(0.0);
  }

  public double getOuterPGPosition() {
    return this.outer_pg.getSelectedSensorPosition();
  }

  public void setOuterPGBasePosition() {
    this.outer_pg.setSelectedSensorPosition(0.0);
  }

  public void setPGInnerSpeed(double pg) {
    this.inner_pg.set(TalonSRXControlMode.PercentOutput, pg * PGConstants.speedMultiplier);
  }

  public void setPGInnerPIDSpeed(double pg) {
    this.inner_pg.set(TalonSRXControlMode.PercentOutput, pg);
  }

  public void setPGOuterPIDSpeed(double pg) {
    this.outer_pg.set(TalonSRXControlMode.PercentOutput, pg);
  }

  public void setPGOuterSpeed(double pg) {
    this.outer_pg.set(TalonSRXControlMode.PercentOutput, pg * ClimberConstants.speedMultiplier);
  }
}
