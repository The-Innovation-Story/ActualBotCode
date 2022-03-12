// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber.pg;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants.PGConstants;

public class OuterPGSubsystem extends SubsystemBase {
  private final WPI_TalonSRX outer_pg;

  /** Creates a new OuterPGSubsystem. */
  public OuterPGSubsystem() {
    this.outer_pg = new WPI_TalonSRX(PGConstants.outerPGConstants);
    this.outer_pg.setSelectedSensorPosition(0.0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("PG Bahar R", this.outer_pg.get());
    // This method will be called once per scheduler run
  }

  public double getOuterPGPosition() {
    return this.outer_pg.getSelectedSensorPosition();
  }

  public void setOuterPGBasePosition() {
    this.outer_pg.setSelectedSensorPosition(0.00);
  }

  public void setPGOuterPIDSpeed(double pg) {
    this.outer_pg.set(TalonSRXControlMode.PercentOutput, pg);
  }

  public void setPGOuterSpeed(double pg) {
    if (Math.abs(pg) > PGConstants.deadband)
      this.outer_pg.set(TalonSRXControlMode.PercentOutput, pg * PGConstants.speedMultiplier);
    else
      this.outer_pg.set(TalonSRXControlMode.PercentOutput, 0.00);
  }
}
