// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.drive;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.auto.feeder.FeederByTimeCommand;
import frc.robot.commands.auto.intake.AutonomousIntakeCommand;
import frc.robot.commands.auto.timepass.AutonomousTimeConsumptionCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutonomousDriveGroupCommand extends SequentialCommandGroup {
  public double INITIAL_DISTANCE = 2;
  public double FEEDER_TIME_ON = 3;
  public double DUPLET_DISTANCE = -1;
  public double DUPLET_ANGLE = -10;
  public double TRIPLET_DISTANCE = 1;
  public double TRIPLET_ANGLE = 10;
  public double TIME_PASS = 2;

  /** Creates a new AutonomousDriveCommandGroup. */
  public AutonomousDriveGroupCommand(DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem,
      FeederSubsystem feederSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new ParallelCommandGroup(
        new AutonomousIntakeCommand(intakeSubsystem),
        new AutonomousDriveTurnCommand(driveSubsystem, INITIAL_DISTANCE, 0),
        new AutonomousTurnByAngleCommand(driveSubsystem, 179)));

    addCommands(new FeederByTimeCommand(feederSubsystem, FEEDER_TIME_ON));

    addCommands(new ParallelCommandGroup(
        new AutonomousTurnByAngleCommand(driveSubsystem, DUPLET_ANGLE),
        new AutonomousDriveTurnCommand(driveSubsystem, DUPLET_DISTANCE, 0)));

    addCommands(new AutonomousTimeConsumptionCommand(driveSubsystem, TIME_PASS));

    addCommands(new ParallelCommandGroup(
        new AutonomousTurnByAngleCommand(driveSubsystem, TRIPLET_ANGLE),
        new AutonomousDriveTurnCommand(driveSubsystem, TRIPLET_DISTANCE, 0)));

    addCommands(new FeederByTimeCommand(feederSubsystem, FEEDER_TIME_ON));
  }
}
