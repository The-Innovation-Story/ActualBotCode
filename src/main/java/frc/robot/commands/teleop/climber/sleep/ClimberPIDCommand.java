// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop.climber.sleep;

import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.climber.pg.PGClimberSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.controller.PIDController;

public class ClimberPIDCommand extends CommandBase {
  private PGClimberSubsystem pgClimberSubsystem;
  private double basePoint;
  private final PIDController innerHolding;

  /** Creates a new ClimberPIDCommand. */
  public ClimberPIDCommand(PGClimberSubsystem pgClimberSubsystem, double basePoint) {
    this.pgClimberSubsystem = pgClimberSubsystem;
    this.basePoint = basePoint;
    this.innerHolding = new PIDController(ClimberConstants.InnerHoldings.kP, ClimberConstants.InnerHoldings.kI,
        ClimberConstants.InnerHoldings.kD);
    this.innerHolding.setSetpoint(this.basePoint);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.pgClimberSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("OPENED 3");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("CLOSED 3");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
