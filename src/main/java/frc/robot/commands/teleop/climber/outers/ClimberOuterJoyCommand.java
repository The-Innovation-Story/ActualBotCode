// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop.climber.outers;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.climber.bahar.OuterClimberSubsystem;

public class ClimberOuterJoyCommand extends CommandBase {
  private OuterClimberSubsystem outerClimberSubsystem;
  private Supplier<Double> outers;
  /** Creates a new ClimberOuterJoyCommand. */
  public ClimberOuterJoyCommand(OuterClimberSubsystem outerClimberSubsystem, Supplier<Double> outers) {
    this.outerClimberSubsystem = outerClimberSubsystem;
    this.outers = outers;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.outerClimberSubsystem);
  }

  public ClimberOuterJoyCommand(OuterClimberSubsystem outerClimberSubsystem2, Supplier<Double> supplier, Object object) {
}

// Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double outers = this.outers.get();

    this.outerClimberSubsystem.setOuterSpeed(outers);
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
