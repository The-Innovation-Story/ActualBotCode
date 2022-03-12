// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShooterByTimeCommand extends ParallelRaceGroup {
  /** Creates a new ShooterByTimeCommand. */
  public ShooterByTimeCommand(ShooterSubsystem shooterSubsystem, double time) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    double now = Timer.getFPGATimestamp();
    addCommands(new AutonomousShooterCommand(shooterSubsystem));
    // System.out.println("UP");
    SmartDashboard.putNumber("Time", now);
    addCommands(new WaitCommand(time));
  }
}
