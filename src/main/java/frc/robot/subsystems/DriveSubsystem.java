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

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  private final CANSparkMax FR;
  private final CANSparkMax BR;
  public RelativeEncoder FR_encoder;
  public RelativeEncoder BR_encoder;
  private final MotorControllerGroup rightSide;
  private final PIDController rightPIDController;

  private final CANSparkMax FL;
  private final CANSparkMax BL;
  public RelativeEncoder FL_encoder;
  public RelativeEncoder BL_encoder;
  private final MotorControllerGroup leftSide;
  private final PIDController leftPIDController;

  private final DifferentialDrive driveTrain;
  private final DifferentialDriveKinematics driveKinematics;
  private final DifferentialDriveOdometry driveOdometry;
  private final SimpleMotorFeedforward driveFeedforward;

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
    this.rightPIDController = new PIDController(DrivingConstants.kP, DrivingConstants.kI, DrivingConstants.kD);

    this.FL = new CANSparkMax(DrivingConstants.FL_ID, MotorType.kBrushless);
    this.BL = new CANSparkMax(DrivingConstants.BL_ID, MotorType.kBrushless);
    this.FL_encoder = this.FL.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor,
        DrivingConstants.neoCountsPerRevolution);
    this.BL_encoder = this.BL.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor,
        DrivingConstants.neoCountsPerRevolution);
    this.leftSide = new MotorControllerGroup(this.FL, this.BL);
    this.rightSide.setInverted(false);
    this.leftPIDController = new PIDController(DrivingConstants.kP, DrivingConstants.kI, DrivingConstants.kD);

    this.driveTrain = new DifferentialDrive(this.leftSide, this.rightSide);
    this.driveTrain.setSafetyEnabled(true);
    this.driveKinematics = new DifferentialDriveKinematics(DrivingConstants.kTrackWidth);
    this.driveOdometry = new DifferentialDriveOdometry(RobotContainer.navx.getRotation2d());
    this.driveFeedforward = new SimpleMotorFeedforward(DrivingConstants.kS, DrivingConstants.kV, DrivingConstants.kA);

    this.FR_encoder.setPosition(0.0);
    this.BR_encoder.setPosition(0.0);
    this.FL_encoder.setPosition(0.0);
    this.BL_encoder.setPosition(0.0);

    this.FR_encoder.setPositionConversionFactor(
        2 * Math.PI * DrivingConstants.kWheelRadius / DrivingConstants.neoCountsPerRevolution);
    this.BR_encoder.setPositionConversionFactor(
        2 * Math.PI * DrivingConstants.kWheelRadius / DrivingConstants.neoCountsPerRevolution);
    this.FL_encoder.setPositionConversionFactor(
        2 * Math.PI * DrivingConstants.kWheelRadius / DrivingConstants.neoCountsPerRevolution);
    this.BL_encoder.setPositionConversionFactor(
        2 * Math.PI * DrivingConstants.kWheelRadius / DrivingConstants.neoCountsPerRevolution);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
    final double leftFeedforward = this.driveFeedforward.calculate(speeds.leftMetersPerSecond);
    final double rightFeedforward = this.driveFeedforward.calculate(speeds.rightMetersPerSecond);

    final double leftOutput = this.leftPIDController.calculate(
        this.FL_encoder.getVelocity() * DrivingConstants.rpm_to_ms_wheel_converter, speeds.leftMetersPerSecond);
    final double rightOutput = this.rightPIDController.calculate(
        this.FR_encoder.getVelocity() * DrivingConstants.rpm_to_ms_wheel_converter, speeds.rightMetersPerSecond);
    this.leftSide.setVoltage(leftOutput + leftFeedforward);
    this.rightSide.setVoltage(rightOutput + rightFeedforward);
  }

  /**
   * Drives the robot with the given linear velocity and angular velocity.
   *
   * @param xSpeed Linear velocity in m/s.
   * @param rot    Angular velocity in rad/s.
   */
  @SuppressWarnings("ParameterName")
  public void drive(double xSpeed, double rot) {
    DifferentialDriveWheelSpeeds wheelSpeeds = this.driveKinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0.0, rot));
    setSpeeds(wheelSpeeds);
    updateOdometry();
  }

  /** Updates the field-relative position. */
  public void updateOdometry() {
    this.driveOdometry.update(
        RobotContainer.navx.getRotation2d(), this.FL_encoder.getPosition(), this.FR_encoder.getPosition());
  }
}
