// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop.climber.pg;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.climber.pg.PGClimberSubsystem;

public class PGClimberCommand extends CommandBase {
  private PGClimberSubsystem pgClimberSubsystem;
  private Supplier<Double> pg;

  /** Creates a new PGClimberCommand. */
  public PGClimberCommand(PGClimberSubsystem pgClimberSubsystem, Supplier<Double> pg) {
    this.pgClimberSubsystem = pgClimberSubsystem;
    this.pg = pg;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.pgClimberSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double pg = this.pg.get();

    this.pgClimberSubsystem.setPGInnerSpeed(pg);
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
