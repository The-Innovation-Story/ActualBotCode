// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber.outer;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants.OuterClimberConstants;

public class OuterClimberSubsystem extends SubsystemBase {
  private final WPI_TalonSRX out_LC, out_RC;
  private final MotorControllerGroup out_C;

  /** Creates a new OuterClimberSubsystem. */
  public OuterClimberSubsystem() {
    this.out_LC = new WPI_TalonSRX(OuterClimberConstants.out_LC_ID);
    this.out_RC = new WPI_TalonSRX(OuterClimberConstants.out_RC_ID);
    this.out_LC.setInverted(false);
    this.out_RC.setInverted(true);

    this.out_C = new MotorControllerGroup(this.out_LC, this.out_RC);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setOuterSpeed(double inners) {
    this.out_C.set(inners * OuterClimberConstants.speedMultiplier);
  }
}
