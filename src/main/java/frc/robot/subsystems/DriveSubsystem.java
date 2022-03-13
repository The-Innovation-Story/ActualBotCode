// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.RobotContainer;
import frc.robot.Constants.DrivingConstants;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
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

  public static AHRS navx;

  private DifferentialDriveOdometry m_odometry;

  private PIDController controller;
  private PIDController controllerang;
  public Pose2d pose;
  public double a_botXpose, b_botYpose;
  public double[] x, y;
  public double angle, angletotake, trans_Lmot, trans_Rmot;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {

    DriveSubsystem.navx = new AHRS(SPI.Port.kMXP);
    // DriveSubsystem.navx.reset();
    DriveSubsystem.navx.zeroYaw();
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
    this.driveTrain.setSafetyEnabled(true);
    this.driveTrain.setDeadband(0.1);
    this.driveTrain.setExpiration(0.1);
    this.driveTrain.setMaxOutput(1);
    // DriveSubsystem.navx.zeroYaw();
    this.m_odometry = new DifferentialDriveOdometry(
        Rotation2d.fromDegrees(-DriveSubsystem.navx.getAngle()));

    this.controller = new PIDController(0.6, 0, 0.00000);
    this.controllerang = new PIDController(0.016, 0, 0.009);

    this.x = new double[] { 2 };
    this.y = new double[] { 0 };

    this.FR_encoder.setPosition(0.0);
    this.BR_encoder.setPosition(0.0);
    this.FL_encoder.setPosition(0.0);
    this.BL_encoder.setPosition(0.0);
  }

  @Override
  public void periodic() {
    this.pose = m_odometry.update(Rotation2d.fromDegrees(-DriveSubsystem.navx.getAngle()),
        (FL_encoder.getPosition() * Math.PI * Units.inchesToMeters(6)) / 7.31,
        (FR_encoder.getPosition() * Math.PI * Units.inchesToMeters(6)) / 7.31);
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("Idhar Dekh : GYRO", DriveSubsystem.navx.getAngle());
    SmartDashboard.putNumber("Idhar Dekh : POSE", this.pose.getRotation().getDegrees());
    SmartDashboard.putNumber("Idhar Dekh : OFFSET", RobotContainer.GYRO_OFFSET);
  }

  public double getPoseAngle() {
    return this.pose.getRotation().getDegrees();
  }

  public void drive(final double l, final double r) {
    this.rightSide.setInverted(true);

    this.FR.set(r);
    this.BR.set(r);
    this.FL.set(l);
    this.BL.set(l);
  }

  public void curvatureDriveInbuilt(final double y, final double z) {
    // System.out.println("Speed: " + y + " " + z);
    this.rightSide.setInverted(false);

    this.driveTrain.curvatureDrive(y, z, false);
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
    return DriveSubsystem.navx.getYaw();
  }

  public void arcadeAutonomousInbuilt(double speed, double turn) {
    this.rightSide.setInverted(false);

    this.driveTrain.arcadeDrive(speed, turn);
  }

  public void setBasePosition() {
    this.FR_encoder.setPosition(0.0);
    this.BR_encoder.setPosition(0.0);
    this.FL_encoder.setPosition(0.0);
    this.BL_encoder.setPosition(0.0);

    DriveSubsystem.navx.reset();
  }

  public double[] speedcontrol(double x, double y) {
    this.a_botXpose = this.pose.getX();
    this.b_botYpose = this.pose.getY();

    this.angle = -DriveSubsystem.navx.getAngle() % 360;
    this.angletotake = Math.toDegrees(Math.atan2((y - this.b_botYpose), (x - this.a_botXpose)));
    // angletotake = Math.toDegrees(Math.atan2(1, 1));

    // Clip the Angle from [-180 to 180] -> [0 to 360]
    this.angletotake = (this.angletotake + 720) % 360;
    double d = Math.sqrt(Math.pow((this.a_botXpose - x), 2) + Math.pow((this.b_botYpose - y), 2));
    double d_theta = this.angletotake - this.angle;
    double theta_dir = d_theta / Math.abs(d_theta);

    if (Math.abs(d_theta) >= 180) {
      d_theta = 360 - Math.abs(d_theta);
      theta_dir *= -1;
    } else {
      d_theta = Math.abs(d_theta);
      theta_dir *= 1;
    }
    SmartDashboard.putNumber("d", d);
    SmartDashboard.putNumber("x", a_botXpose);
    SmartDashboard.putNumber("y", b_botYpose);
    SmartDashboard.putNumber("angle to take", angletotake);
    SmartDashboard.putNumber("angle to take with direction", d_theta *
        theta_dir);
    SmartDashboard.putNumber("D Theta", d_theta);
    SmartDashboard.putNumber("Dir", theta_dir);
    SmartDashboard.putNumber("angle", angle);
    SmartDashboard.putNumber("Robotnavx", DriveSubsystem.navx.getAngle());
    SmartDashboard.putNumber("speed", controllerang.calculate(0, d_theta *
        theta_dir));
    if (Math.abs(x - this.a_botXpose) > 0.2 || Math.abs(y - this.b_botYpose) > 0.2) {
      this.trans_Lmot = this.controller.calculate(0, d) - this.controllerang.calculate(0, d_theta * theta_dir);
      this.trans_Rmot = this.controller.calculate(0, d) + this.controllerang.calculate(0, d_theta * theta_dir);
    } else {
      this.trans_Lmot = 0;
      this.trans_Rmot = 0;
    }

    // Equal Weightage for Both PIDs
    if (Math.abs(this.trans_Lmot) > 0.1 || Math.abs(this.trans_Rmot) > 0.1) {
      if (Math.abs(this.trans_Lmot) > Math.abs(this.trans_Rmot)) {
        this.trans_Rmot = 0.1 * this.trans_Rmot / Math.abs(this.trans_Lmot);
        this.trans_Lmot = 0.1 * this.trans_Lmot / Math.abs(this.trans_Lmot);
        // MathUtil.F(trans_Lmot, -0.2, 0.2);
      } else {
        this.trans_Lmot = 0.1 * this.trans_Lmot / Math.abs(this.trans_Rmot);
        this.trans_Rmot = 0.1 * this.trans_Rmot / Math.abs(this.trans_Rmot);
        // MathUtil.clamp(trans_Rmot, -0.2, 0.2);
      }

    }
    double speed[] = { this.trans_Lmot, this.trans_Rmot };
    // double speed[] = { 0.1, 0.1 };

    return speed;
    // Ang_Lmot=
    // Ang_Rmot=
  }

  public double getA_Xpose() {
    return a_botXpose;
  }

  public double getB_Ypose() {
    return b_botYpose;
  }

  public void setSpeeds(double[] speeds) {
    this.leftSide.set(speeds[0]);
    this.rightSide.set(speeds[1]);
  }

}
