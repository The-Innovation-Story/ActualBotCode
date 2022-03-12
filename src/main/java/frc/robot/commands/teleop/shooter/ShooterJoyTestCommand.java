// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop.shooter;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterJoyTestCommand extends CommandBase {
  private ShooterSubsystem shooterSubsystem;
  private Supplier<Double> shooter_axis;

  /** Creates a new ShooterJoyTestCommand. */
  public ShooterJoyTestCommand(ShooterSubsystem shooterSubsystem, Supplier<Double> shooter_axis) {
    this.shooterSubsystem = shooterSubsystem;
    this.shooter_axis = shooter_axis;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double shoot = this.shooter_axis.get();

    this.shooterSubsystem.setSpeed(shoot);
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
