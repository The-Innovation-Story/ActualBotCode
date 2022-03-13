// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.drive;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.Robot;
import frc.robot.subsystems.DriveSubsystem;

public class AutonomousTurnByAngleCommand extends CommandBase {
  private final DriveSubsystem driveSubsystem;
  private double angle;

  /** Creates a new AutonomousTurnByAngleCommand. */
  public AutonomousTurnByAngleCommand(DriveSubsystem driveSubsystem, double angle) {
    this.driveSubsystem = driveSubsystem;
    this.angle = angle;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    DriveSubsystem.navx.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("YAW", DriveSubsystem.navx.getYaw());
    SmartDashboard.putNumber("ANGLE", DriveSubsystem.navx.getAngle());

    // this.driveSubsystem.arcadeInbuilt(0.0,
    // (Math.signum(this.angle) * (Math.abs(Math.abs(this.angle) -
    // Math.abs(DriveSubsystem.navx.getYaw())))) * 0.003);
    this.driveSubsystem.drive(
        Math.signum(this.angle) * Math.abs(Math.abs(this.angle) - Math.abs(DriveSubsystem.navx.getYaw())) * 0.006,
        -1 * (Math.signum(this.angle) * (Math.abs(Math.abs(this.angle) - Math.abs(DriveSubsystem.navx.getYaw()))))
            * 0.006);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.driveSubsystem.drive(0, 0);
    DriveSubsystem.navx.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(DriveSubsystem.navx.getAngle()) >= Math.abs(this.angle));
  }
}
