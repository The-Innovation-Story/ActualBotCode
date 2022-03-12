// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop.climber.pg.outer;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ClimberConstants.PGConstants;
import frc.robot.subsystems.climber.pg.OuterPGSubsystem;

public class OuterPGClimberStopCommand extends CommandBase {
  private OuterPGSubsystem outerPGSubsystem;
  private double basePoint;
  private Supplier<Boolean> endMove;
  private final PIDController outerHolding;
  /** Creates a new OuterPGClimberStopCommand. */
  public OuterPGClimberStopCommand(OuterPGSubsystem outerPGSubsystem, double basePoint, Supplier<Boolean> endMove) {
    this.outerPGSubsystem = outerPGSubsystem;
    this.basePoint = basePoint;
    this.endMove = endMove;
    this.outerHolding = new PIDController(PGConstants.kPOuter, PGConstants.kIOuter, PGConstants.kDOuter);

    this.outerHolding.setSetpoint(this.basePoint);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.outerPGSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = MathUtil.clamp(this.outerHolding.calculate(
        this.outerPGSubsystem.getOuterPGPosition(),
        this.basePoint),
        -0.75, 0.75);

    SmartDashboard.putNumber("Current Outer Position", this.outerPGSubsystem.getOuterPGPosition());

    this.outerPGSubsystem.setPGOuterPIDSpeed(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.outerPGSubsystem.setPGOuterPIDSpeed(0.0);
    this.outerPGSubsystem.setOuterPGBasePosition();
    this.outerHolding.close();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.endMove.get();
  }
}
