// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.vision;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.DriveSubsystem;

public class LimelightAlignCommand extends CommandBase {
  private DriveSubsystem driveSubsystem;
  private final ProfiledPIDController turnController;

  /** Creates a new LimelightAlignCommand. */
  public LimelightAlignCommand(DriveSubsystem driveSubsystem) {
    this.driveSubsystem = driveSubsystem;
    this.turnController = new ProfiledPIDController(VisionConstants.kP, VisionConstants.kI, VisionConstants.kD,
        new TrapezoidProfile.Constraints(VisionConstants.kMaxVelocityTurning,
            VisionConstants.kMaxAccelerationTurning));

    this.turnController.setTolerance(0.2);
    this.turnController.setGoal(0.0);
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
    double turn = MathUtil.clamp(this.turnController.calculate(RobotContainer.getDistanceGyroSeparationFromGoal(), 0.0),
        VisionConstants.minTurnSpeed, VisionConstants.maxTurnSpeed);
    this.driveSubsystem.arcadeAutonomousInbuilt(0.0, turn);
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
