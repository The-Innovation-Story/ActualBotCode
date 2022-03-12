// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber.pg;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants.PGConstants;

public class InnerPGSubsystem extends SubsystemBase {
  private final WPI_TalonSRX inner_pg;

  /** Creates a new InnerPGSubsystem. */
  public InnerPGSubsystem() {
    this.inner_pg = new WPI_TalonSRX(PGConstants.innerPGConstants);
    this.inner_pg.setSelectedSensorPosition(0.0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("PG Andar L", this.inner_pg.get());
    // This method will be called once per scheduler run
  }

  public void setPGInnerSpeed(double pg) {
    if (Math.abs(pg) > PGConstants.deadband)
      this.inner_pg.set(TalonSRXControlMode.PercentOutput, pg * PGConstants.speedMultiplier);
    else
      this.inner_pg.set(TalonSRXControlMode.PercentOutput, 0.00);
  }

  public void setPGInnerPIDSpeed(double pg) {
    this.inner_pg.set(TalonSRXControlMode.PercentOutput, pg);
  }

  public double getInnerPGPosition() {
    return this.inner_pg.getSelectedSensorPosition();
  }

  public void setInnerPGBasePosition() {
    this.inner_pg.setSelectedSensorPosition(0.0);
  }
}
