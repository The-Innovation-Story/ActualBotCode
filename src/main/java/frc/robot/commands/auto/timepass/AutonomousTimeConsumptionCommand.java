// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.timepass;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class AutonomousTimeConsumptionCommand extends CommandBase {
  private DriveSubsystem driveSubsystem;
  private double timer, now, titit;

  /** Creates a new AutonomousTimeConsumptionCommand. */
  public AutonomousTimeConsumptionCommand(DriveSubsystem driveSubsystem, double time) {
    this.driveSubsystem = driveSubsystem;
    this.timer = time;
    this.now = 0.0;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.titit = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.now = Timer.getFPGATimestamp();
    this.driveSubsystem.drive(0.1, 0.1);
    SmartDashboard.putNumber("NOWXXXX", this.now - titit);
    System.out.println(this.now - titit);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (this.now > this.timer);
  }
}
