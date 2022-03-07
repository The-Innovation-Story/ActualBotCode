// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop.climber.sleep;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.climber.andar.InnerClimberSubsystem;

public class ClimberInnersTimedMovementCommand extends CommandBase {
  private InnerClimberSubsystem innerClimberSubsystem;
  private double time, now;

  /** Creates a new ClimberInnersTimedMovementCommand. */
  public ClimberInnersTimedMovementCommand(InnerClimberSubsystem innerClimberSubsystem, double time) {
    this.innerClimberSubsystem = innerClimberSubsystem;
    this.time = time;
    this.now = 0.0;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.innerClimberSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // this.innerClimberSubsystem.setRunToTimer(this.time);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.now = Timer.getFPGATimestamp();
    this.innerClimberSubsystem.setInnerSpeed(0.45);
    SmartDashboard.putNumber("NOWSS", this.now);
    System.out.println(now);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.innerClimberSubsystem.setInnerSpeed(0.0);
    // this.innerClimberSubsystem.andarWale_Break();
    System.out.println("CLOSED 2");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (this.now > this.time);
    // return false;
  }
}
