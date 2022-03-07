// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop.climber.pg;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ClimberConstants.PGConstants;
import frc.robot.subsystems.climber.pg.PGClimberSubsystem;

public class InnerPGClimberStopCommand extends CommandBase {
  private PGClimberSubsystem pgClimberSubsystem;
  private double basePoint;
  private Supplier<Boolean> endMove;
  private final PIDController innerHolding;

  /** Creates a new InnerPGClimberStopCommand. */
  public InnerPGClimberStopCommand(PGClimberSubsystem pgClimberSubsystem, double basePoint, Supplier<Boolean> endMove) {
    this.pgClimberSubsystem = pgClimberSubsystem;
    this.basePoint = basePoint;
    this.endMove = endMove;
    this.innerHolding = new PIDController(PGConstants.kPInner, PGConstants.kIInner, PGConstants.kDInner);

    this.innerHolding.setSetpoint(this.basePoint);
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
    double speed = MathUtil.clamp(this.innerHolding.calculate(
        this.pgClimberSubsystem.getInnerPGPosition(),
        this.basePoint),
        -0.75, 0.75);

    SmartDashboard.putNumber("Current Inner Position", this.pgClimberSubsystem.getInnerPGPosition());

    this.pgClimberSubsystem.setPGInnerPIDSpeed(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.pgClimberSubsystem.setPGInnerPIDSpeed(0.0);
    this.pgClimberSubsystem.setInnerPGBasePosition();
    this.innerHolding.close();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.endMove.get();
  }
}
