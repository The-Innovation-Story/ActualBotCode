// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.timepass;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class AutonomousTimeConsumptionCommand extends CommandBase {
  private DriveSubsystem driveSubsystem;
  private double timer, now;
  /** Creates a new AutonomousTimeConsumptionCommand. */
  public AutonomousTimeConsumptionCommand(DriveSubsystem driveSubsystem, double time) {
    this.driveSubsystem = driveSubsystem;
    this.timer = time; 
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.now = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.driveSubsystem.drive(0.0, 0.0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (this.now > this.timer);
  }
}
