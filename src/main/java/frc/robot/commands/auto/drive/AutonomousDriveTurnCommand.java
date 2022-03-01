// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DrivingConstants;
import frc.robot.subsystems.DriveSubsystem;

public class AutonomousDriveTurnCommand extends CommandBase {
  private DriveSubsystem driveSubsystem;
  private final ProfiledPIDController driveController, turnController;
  private double speed, turn, distance, angle;

  /** Creates a new AutonomousDriveTurnCommand. */
  public AutonomousDriveTurnCommand(DriveSubsystem driveSubsystem, double distance, double angle) {
    this.driveSubsystem = driveSubsystem;
    this.distance = distance;
    this.angle = angle;
    this.driveController = new ProfiledPIDController(DrivingConstants.kP, DrivingConstants.kI, DrivingConstants.kD,
        new TrapezoidProfile.Constraints(DrivingConstants.kMaxVelocity, DrivingConstants.kMaxAcceleration));
    this.turnController = new ProfiledPIDController(DrivingConstants.kPTurn, DrivingConstants.kITurn,
        DrivingConstants.kDTurn, new TrapezoidProfile.Constraints(DrivingConstants.kMaxVelocityTurning,
            DrivingConstants.kMaxAccelerationTurning));

    this.driveSubsystem.setBasePosition();
    this.turnController.enableContinuousInput(-180, 180);
    // The integral gain term will never add or subtract more than 0.5 from the
    // total loop output
    // Kindly Set This as per our use case (ye safety ke liye rakha hai)
    this.driveController.setIntegratorRange(-0.5, 0.5);
    this.turnController.setIntegratorRange(-0.5, 0.5);
    this.driveController.setGoal(distance);
    this.turnController.setGoal(angle);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.driveController.reset(0.0);
    this.turnController.reset(0.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.speed = MathUtil.clamp(this.driveController.calculate(this.driveSubsystem.getEncoderDistance(), distance),
        DrivingConstants.kMinimumAutonomousDriveSpeed, DrivingConstants.kMaximumAutonomousDriveSpeed);
    this.turn = MathUtil.clamp(this.turnController.calculate(this.driveSubsystem.getHeading(), angle),
        DrivingConstants.kMinimumAutonomousTurnSpeed, DrivingConstants.kMaximumAutonomousTurnSpeed);

    this.driveSubsystem.arcadeAutonomousInbuilt(this.speed, this.turn);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.driveSubsystem.drive(0.0, 0.0);
    this.driveSubsystem.setBasePosition();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
