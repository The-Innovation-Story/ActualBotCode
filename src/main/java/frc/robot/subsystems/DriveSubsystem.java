// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.RobotContainer;
import frc.robot.Constants.DrivingConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  private CANSparkMax FR;
  private CANSparkMax BR;
  public RelativeEncoder FR_encoder;
  public RelativeEncoder BR_encoder;
  private MotorControllerGroup rightSide;

  private CANSparkMax FL;
  private CANSparkMax BL;
  public RelativeEncoder FL_encoder;
  public RelativeEncoder BL_encoder;
  private MotorControllerGroup leftSide;

  private DifferentialDrive driveTrain;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    this.FR = new CANSparkMax(DrivingConstants.FR_ID, MotorType.kBrushless);
    this.BR = new CANSparkMax(DrivingConstants.BR_ID, MotorType.kBrushless);
    this.FR_encoder = this.FR.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor,
        DrivingConstants.neoCountsPerRevolution);
    this.BR_encoder = this.BR.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor,
        DrivingConstants.neoCountsPerRevolution);
    this.rightSide = new MotorControllerGroup(this.FR, this.BR);
    this.rightSide.setInverted(true);

    this.FL = new CANSparkMax(DrivingConstants.FL_ID, MotorType.kBrushless);
    this.BL = new CANSparkMax(DrivingConstants.BL_ID, MotorType.kBrushless);
    this.FL_encoder = this.FL.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor,
        DrivingConstants.neoCountsPerRevolution);
    this.BL_encoder = this.BL.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor,
        DrivingConstants.neoCountsPerRevolution);
    this.leftSide = new MotorControllerGroup(this.FL, this.BL);
    this.leftSide.setInverted(false);

    this.driveTrain = new DifferentialDrive(this.leftSide, this.rightSide);
    // this.driveTrain.setSafetyEnabled(false);

    this.FR_encoder.setPosition(0.0);
    this.BR_encoder.setPosition(0.0);
    this.FL_encoder.setPosition(0.0);
    this.BL_encoder.setPosition(0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void drive(final double l, final double r) {
    this.rightSide.setInverted(true);

    this.FR.set(r);
    this.BR.set(r);
    this.FL.set(l);
    this.BL.set(l);
  }

  public void arcadeInbuilt(final double y, final double z) {
    // System.out.println("Speed: " + y + " " + z);
    this.rightSide.setInverted(false);

    this.driveTrain.arcadeDrive(y * DrivingConstants.kMaxSpeed, z * DrivingConstants.kMaxAngularSpeed);
  }

  public double getEncoderDistance() {
    double distance = (FL_encoder.getPosition() + FR_encoder.getPosition() + BL_encoder.getPosition()
        + BR_encoder.getPosition()) / 4;
    return (distance * Math.PI * Units.inchesToMeters(6)) / 7.31;
  }

  public double getHeading() {
    return RobotContainer.navx.getYaw();
  }

  public void arcadeAutonomousInbuilt(double speed, double turn) {
    this.rightSide.setInverted(false);

    SmartDashboard.putNumber("Speed", speed);
    SmartDashboard.putNumber("Turn", turn);
    this.driveTrain.arcadeDrive(speed, turn);
  }

  public void setBasePosition() {
    this.FR_encoder.setPosition(0.0);
    this.BR_encoder.setPosition(0.0);
    this.FL_encoder.setPosition(0.0);
    this.BL_encoder.setPosition(0.0);

    RobotContainer.navx.reset();
  }

}
