// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop.climber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.climber.pg.PGClimberSubsystem;

public class ClimberHoldingOutersCommand extends CommandBase {
  private PGClimberSubsystem pgClimberSubsystem;
  private double basePoint;
  private boolean moveUp;
  private final PIDController innerHolding;

  /** Creates a new ClimberHoldingOutersCommand. */
  public ClimberHoldingOutersCommand(PGClimberSubsystem pgClimberSubsystem, double basePoint, boolean moveUp) {
    this.pgClimberSubsystem = pgClimberSubsystem;
    this.moveUp = moveUp;
    this.basePoint = basePoint;
    this.innerHolding = new PIDController(ClimberConstants.InnerHoldings.kP, ClimberConstants.InnerHoldings.kI,
        ClimberConstants.InnerHoldings.kD);

    this.innerHolding.setSetpoint(
        this.basePoint
            + (this.moveUp ? -1 : 1) * (this.moveUp ? ClimberConstants.InnerHoldings.innerUpperPGPositionDifference
                : ClimberConstants.InnerHoldings.innerDownerPGPositionDifference));
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.pgClimberSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("OPENED G1");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = MathUtil.clamp(
        this.innerHolding.calculate(
            this.pgClimberSubsystem.getInnerPGPosition()
                + (this.moveUp ? -1 : 1) * (this.moveUp ? ClimberConstants.InnerHoldings.innerUpperPGPositionDifference
                    : ClimberConstants.InnerHoldings.innerDownerPGPositionDifference),
            this.basePoint
                + (this.moveUp ? -1 : 1) * (this.moveUp ? ClimberConstants.InnerHoldings.innerUpperPGPositionDifference
                    : ClimberConstants.InnerHoldings.innerDownerPGPositionDifference)),
        -0.9, 0.9);

    this.pgClimberSubsystem.setPGOuterPIDSpeed(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("CLOSED G1");
    this.pgClimberSubsystem.setPGInnerPIDSpeed(0.0);
    this.pgClimberSubsystem.setInnerPGBasePosition();
    this.innerHolding.close();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (this.moveUp
        ? (this.pgClimberSubsystem.getInnerPGPosition()
            - this.basePoint) > ClimberConstants.InnerHoldings.innerUpperPGPositionDifference
        : (this.pgClimberSubsystem.getInnerPGPosition()
            - this.basePoint) < ClimberConstants.InnerHoldings.innerDownerPGPositionDifference);
  }
}
