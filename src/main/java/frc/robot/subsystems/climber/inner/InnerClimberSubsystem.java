// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber.inner;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants.InnerClimberConstants;

public class InnerClimberSubsystem extends SubsystemBase {
  private final CANSparkMax in_LC, in_RC;
  private final MotorControllerGroup in_C;

  /** Creates a new InnerClimberSubsystem. */
  public InnerClimberSubsystem() {
    this.in_LC = new CANSparkMax(InnerClimberConstants.in_LC_ID, MotorType.kBrushless);
    this.in_RC = new CANSparkMax(InnerClimberConstants.in_RC_ID, MotorType.kBrushless);

    this.in_C = new MotorControllerGroup(this.in_LC, this.in_RC);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void inners_Break() {
    this.in_LC.setIdleMode(IdleMode.kBrake);
    this.in_RC.setIdleMode(IdleMode.kBrake);
  }

  public void inners_Coast() {
    this.in_LC.setIdleMode(IdleMode.kCoast);
    this.in_RC.setIdleMode(IdleMode.kCoast);
  }

  public void setInnerSpeed(double outers) {
    this.in_C.set(outers * InnerClimberConstants.speedMultiplier);
  }
}
