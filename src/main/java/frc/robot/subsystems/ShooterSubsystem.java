// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  private final CANSparkMax shooter;
  private final SparkMaxPIDController shootPIDController;
  public static RelativeEncoder shootEncoder;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    this.shooter = new CANSparkMax(ShooterConstants.shooter_ID, MotorType.kBrushless);
    this.shooter.restoreFactoryDefaults();
    this.shootPIDController = this.shooter.getPIDController();
    ShooterSubsystem.shootEncoder = this.shooter.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor,
        ShooterConstants.neoCountsPerRevolution);

    this.shootPIDController.setP(ShooterConstants.kP);
    this.shootPIDController.setI(ShooterConstants.kI);
    this.shootPIDController.setD(ShooterConstants.kD);
    this.shootPIDController.setIZone(ShooterConstants.kIz);
    this.shootPIDController.setFF(ShooterConstants.kFF);
    this.shootPIDController.setOutputRange(ShooterConstants.kMinOutput, ShooterConstants.kMaxOutput);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void shootReference(double rpm) {
    SmartDashboard.putNumber("RPM", rpm);
    this.shootPIDController.setReference(rpm, CANSparkMax.ControlType.kVelocity);
  }

  public void stopShooter() {
    this.shooter.set(0);
  }

  public void setSpeed() {
    double speedmotor = shootEncoder.getVelocity();
    if (Math.abs(speedmotor) < ShooterConstants.deadbandVelocity) {
      this.shootPIDController.setReference(ShooterConstants.setThisVelocity, CANSparkMax.ControlType.kVelocity);
      // SmartDashboard.putNumber("POSITION",
      // ShooterSubsystem.shootEncoder.getPosition());
      SmartDashboard.putNumber("VELOCITY", ShooterSubsystem.shootEncoder.getPosition());
    } else
      this.shooter.set(0);
  }
}
