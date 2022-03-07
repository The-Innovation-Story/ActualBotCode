// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop.climber.inner;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.climber.inner.InnerClimberSubsystem;

public class InnerClimberCommand extends CommandBase {
  private InnerClimberSubsystem innerClimberSubsystem;
  private Supplier<Double> inners;
  /** Creates a new InnerClimberCommand. */
  public InnerClimberCommand(InnerClimberSubsystem innerClimberSubsystem, Supplier<Double> inners) {
    this.innerClimberSubsystem = innerClimberSubsystem;
    this.inners = inners;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.innerClimberSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double inners = this.inners.get();

    this.innerClimberSubsystem.setInnerSpeed(inners);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
