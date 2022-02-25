// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DrivingConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.teleop.DriveCommand;
import frc.robot.subsystems.DriveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // IO Devices
  public static AHRS navx;
  public static Joystick joyD, joyC;

  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem driveSubsystem;
  private final SlewRateLimiter speedLimit, turnLimit;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    this.driveSubsystem = new DriveSubsystem();

    this.speedLimit = new SlewRateLimiter(DrivingConstants.kRiseLimiter);
    this.turnLimit = new SlewRateLimiter(DrivingConstants.kRiseLimiter);

    RobotContainer.navx = new AHRS(SPI.Port.kMXP);
    RobotContainer.joyD = new Joystick(OIConstants.kDriverJoystickPort);
    RobotContainer.joyC = new Joystick(OIConstants.kClimberJoystickPort);
    // Configure the button bindings
    configureButtonBindings();

    // Set Default commands
    this.driveSubsystem.setDefaultCommand(
        new DriveCommand(this.driveSubsystem, () -> -1 * RobotContainer.joyD.getRawAxis(OIConstants.kJoyDSpeedAxis),
            () -> RobotContainer.joyD.getRawAxis(OIConstants.kJoyDTurnAxis), this.speedLimit, this.turnLimit));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
}
