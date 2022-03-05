// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.drive;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutonomousDriveRoutineGroupCommand extends SequentialCommandGroup {
  public double INITIAL_DISTANCE = 2;
  public double DUPLET_DISTANCE = -1;
  public double DUPLET_ANGLE = -10;
  public double TRIPLET_DISTANCE = 1;
  public double TRIPLET_ANGLE = 10;

  /** Creates a new AutonomousDriveRoutineGroupCommand. */
  public AutonomousDriveRoutineGroupCommand(DriveSubsystem driveSubsystem) {

    addCommands(new AutonomousDriveTurnCommand(driveSubsystem, INITIAL_DISTANCE, 0));
    addCommands(new AutonomousTurnByAngleCommand(driveSubsystem, 179));
    addCommands(new AutonomousDriveTurnCommand(driveSubsystem, DUPLET_DISTANCE, DUPLET_ANGLE));
    addCommands(new AutonomousDriveTurnCommand(driveSubsystem, TRIPLET_DISTANCE, TRIPLET_ANGLE));
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
  }
}
