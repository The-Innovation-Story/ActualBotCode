// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber.andar;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxRelativeEncoder.Type;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class InnerClimberSubsystem extends SubsystemBase {
  private final CANSparkMax baharWala_L, baharWala_R;
  public final RelativeEncoder baharEncoder_L, baharEncoder_R;
  private final MotorControllerGroup outerWare;

  /** Creates a new OuterClimberSubsystem. */
  public InnerClimberSubsystem() {
    this.baharWala_L = new CANSparkMax(31, MotorType.kBrushless);
    this.baharWala_R = new CANSparkMax(32, MotorType.kBrushless);
    this.baharWala_L.setIdleMode(IdleMode.kBrake);
    this.baharWala_R.setIdleMode(IdleMode.kBrake);
    this.baharEncoder_L = this.baharWala_L.getEncoder(Type.kHallSensor, 42);
    this.baharEncoder_R = this.baharWala_R.getEncoder(Type.kHallSensor, 42);
    this.outerWare = new MotorControllerGroup(this.baharWala_L, this.baharWala_R);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void andarWale_Break() {
    this.baharWala_L.setIdleMode(IdleMode.kBrake);
    this.baharWala_R.setIdleMode(IdleMode.kBrake);
  }

  public void andarWale_Coast() {
    this.baharWala_L.setIdleMode(IdleMode.kCoast);
    this.baharWala_R.setIdleMode(IdleMode.kCoast);
  }

  public void setInnerSpeed(double outers) {
    this.outerWare.set(outers * ClimberConstants.speedMultiplier);
  }
}
