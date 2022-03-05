// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.drive;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.auto.shooter.AutonomousShooterCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutonomousBaseDriveShootCommandGroup extends ParallelCommandGroup {
  /** Creates a new AutonomousBaseDriveShootCommandGroup. */
  public AutonomousBaseDriveShootCommandGroup(ShooterSubsystem shooterSubsystem, DriveSubsystem driveSubsystem,
      IntakeSubsystem intakeSubsystem, FeederSubsystem feederSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new AutonomousShooterCommand(shooterSubsystem),
        new AutonomousDriveGroupCommand(driveSubsystem, intakeSubsystem, feederSubsystem));
  }
}
