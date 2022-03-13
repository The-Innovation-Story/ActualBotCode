// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop.drive;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;

public class DriveCommand extends CommandBase {
  private DriveSubsystem driveSubsystem;
  private Supplier<Double> speed, turn;
  private SlewRateLimiter speedLimit, turnLimit;

  /** Creates a new DriveCommand. */
  public DriveCommand(DriveSubsystem driveSubsystem, Supplier<Double> speed, Supplier<Double> turn,
      SlewRateLimiter speedLimit, SlewRateLimiter turnLimit) {
    this.driveSubsystem = driveSubsystem;
    this.speed = speed;
    this.turn = turn;
    this.speedLimit = speedLimit;
    this.turnLimit = turnLimit;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = this.speedLimit.calculate(this.speed.get());
    double turn = this.turnLimit.calculate(this.turn.get());

    SmartDashboard.putNumber("Limelight X", RobotContainer.getDistanceGyroSeparationFromGoal());
    SmartDashboard.putNumber("Limelight Y", RobotContainer.getDistanceToGoal());

    SmartDashboard.putNumber("D Idhar Dekh : GYRO", DriveSubsystem.navx.getAngle());
    SmartDashboard.putNumber("D Idhar Dekh : POSE", this.driveSubsystem.getPoseAngle());
    SmartDashboard.putNumber("D Idhar Dekh : OFFSET", RobotContainer.GYRO_OFFSET);

    this.driveSubsystem.arcadeInbuilt(speed, turn);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
