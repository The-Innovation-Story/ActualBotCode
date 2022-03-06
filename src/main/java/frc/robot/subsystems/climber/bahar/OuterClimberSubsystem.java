// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber.bahar;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class OuterClimberSubsystem extends SubsystemBase {
  private final WPI_TalonSRX andarWala_L, andarWala_R;
  private final MotorControllerGroup innerWare;

  /** Creates a new InnerClimberSubsystem. */
  public OuterClimberSubsystem() {
    this.andarWala_L = new WPI_TalonSRX(5);
    this.andarWala_R = new WPI_TalonSRX(7);
    this.andarWala_L.setInverted(false);
    this.andarWala_R.setInverted(true);
    this.andarWala_L.setSelectedSensorPosition(0.0);
    this.andarWala_R.setSelectedSensorPosition(0.0);
    this.innerWare = new MotorControllerGroup(this.andarWala_L, this.andarWala_R);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setOuterSpeed(double inners) {
    this.innerWare.set(inners * ClimberConstants.speedMultiplier);
  }
}
