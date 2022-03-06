// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop.climber.inners;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.teleop.climber.ClimberHoldingInnersCommand;
import frc.robot.commands.teleop.climber.sleep.ClimberInnersTimedMovementCommand;
import frc.robot.commands.teleop.climber.sleep.ClimberPIDCommand;
import frc.robot.subsystems.climber.andar.InnerClimberSubsystem;
import frc.robot.subsystems.climber.pg.PGClimberSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ClimberInnerGroupStageCommand extends SequentialCommandGroup {
  /** Creates a new ClimberInnerGroupStageCommand. */
  public ClimberInnerGroupStageCommand(PGClimberSubsystem pgClimberSubsystem,
      InnerClimberSubsystem innerClimberSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    // addCommands();

    addCommands(new ClimberHoldingInnersCommand(pgClimberSubsystem, pgClimberSubsystem.getInnerPGPosition(), true));

    addCommands(new ClimberInnersTimedMovementCommand(innerClimberSubsystem, 1.0));

    addCommands(new ClimberPIDCommand(pgClimberSubsystem, pgClimberSubsystem.getInnerPGPosition()));
  }
}
