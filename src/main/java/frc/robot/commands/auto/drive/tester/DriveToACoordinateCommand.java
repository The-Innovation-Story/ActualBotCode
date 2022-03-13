// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.drive.tester;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class DriveToACoordinateCommand extends CommandBase {
  private DriveSubsystem driveSubsystem;
  private double x, y;

  /** Creates a new DriveToACoordinateCommand. */
  public DriveToACoordinateCommand(DriveSubsystem driveSubsystem, double x, double y) {
    this.driveSubsystem = driveSubsystem;
    this.x = x;
    this.y = y;
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
    this.driveSubsystem.setSpeeds(this.driveSubsystem.speedcontrol(x, y));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.driveSubsystem.setSpeeds(new double[] { 0, 0 });
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(x - this.driveSubsystem.getA_Xpose()) < 0.1
        && Math.abs(y - this.driveSubsystem.getB_Ypose()) < 0.1);
  }
}
