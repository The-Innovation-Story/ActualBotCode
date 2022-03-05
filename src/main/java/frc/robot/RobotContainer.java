// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.DrivingConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.VisionConstants;
// import frc.robot.commands.auto.drive.AutonomousDriveRoutineGroupCommand;
import frc.robot.commands.auto.drive.AutonomousTurnByAngleCommand;
import frc.robot.commands.teleop.drive.DriveCommand;
import frc.robot.commands.teleop.feeder.FeederCommand;
import frc.robot.commands.teleop.intake.IntakeCommand;
import frc.robot.commands.teleop.intake.IntakeStoppingCommand;
import frc.robot.commands.teleop.shooter.ShooterCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

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
  public static Joystick joyD;
  public static NetworkTable table;
  public static NetworkTableEntry tv, tx, ty, ta;

  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem driveSubsystem;
  private final IntakeSubsystem intakeSubsystem;
  private final FeederSubsystem feederSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final SlewRateLimiter speedLimit, turnLimit;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    this.driveSubsystem = new DriveSubsystem();
    this.intakeSubsystem = new IntakeSubsystem();
    this.feederSubsystem = new FeederSubsystem();
    this.shooterSubsystem = new ShooterSubsystem();

    this.speedLimit = new SlewRateLimiter(DrivingConstants.kRiseLimiter);
    this.turnLimit = new SlewRateLimiter(DrivingConstants.kRiseLimiter);

    RobotContainer.navx = new AHRS(SPI.Port.kMXP);
    RobotContainer.joyD = new Joystick(OIConstants.kDriverJoystickPort);
    // RobotContainer.joyC = new Joystick(OIConstants.kClimberJoystickPort);
    RobotContainer.table = NetworkTableInstance.getDefault().getTable(VisionConstants.limelight);
    RobotContainer.tv = RobotContainer.table.getEntry(VisionConstants.tv);
    RobotContainer.tx = RobotContainer.table.getEntry(VisionConstants.tx);
    RobotContainer.ty = RobotContainer.table.getEntry(VisionConstants.ty);
    RobotContainer.ta = RobotContainer.table.getEntry(VisionConstants.ta);
    // Configure the button bindings
    configureButtonBindings();

    // Set Default commands
    // Drive
    this.driveSubsystem.setDefaultCommand(
        new DriveCommand(this.driveSubsystem, () -> -1 * RobotContainer.joyD.getRawAxis(OIConstants.kJoyDSpeedAxis),
            () -> RobotContainer.joyD.getRawAxis(OIConstants.kJoyDTurnAxis), this.speedLimit, this.turnLimit));

    // Intake
    this.intakeSubsystem.setDefaultCommand(new IntakeStoppingCommand(this.intakeSubsystem));
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
    // Feeder Button Integration
    new JoystickButton(RobotContainer.joyD, OIConstants.feeder_X_ButtonNumber)
        .debounce(OIConstants.feederDebouncePeriod).whenActive(new FeederCommand(this.feederSubsystem));

    // Intake Forward Button Integration
    new JoystickButton(RobotContainer.joyD, OIConstants.intakeForward_Y_ButtonNumber)
        .debounce(OIConstants.feederDebouncePeriod).toggleWhenActive(new IntakeCommand(this.intakeSubsystem));

    // Shooter Button Binding Integration
    new JoystickButton(RobotContainer.joyD, OIConstants.shooter_RB_ButtonNumber)
        .debounce(OIConstants.feederDebouncePeriod).whenActive(new ShooterCommand(this.shooterSubsystem));

    // Turn An Angle
    new JoystickButton(RobotContainer.joyD, OIConstants.turn_LB_ButtonNumber)
        .debounce(OIConstants.feederDebouncePeriod).whenActive(new ShooterCommand(this.shooterSubsystem));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    // return new AutonomousDriveRoutineGroupCommand(this.driveSubsystem);
    return new AutonomousTurnByAngleCommand(this.driveSubsystem, 100);
  }

  public static boolean getTarget() {
    RobotContainer.tv = RobotContainer.table.getEntry(VisionConstants.tv);
    return tv.getBoolean(false);
  }

  public double getDistanceToGoal() {
    RobotContainer.ty = RobotContainer.table.getEntry(VisionConstants.ty);
    return ty.getDouble(VisionConstants.defaultValue);
  }

  public static double getDistanceGyroSeparationFromGoal() {
    RobotContainer.tx = RobotContainer.table.getEntry(VisionConstants.tx);
    return tx.getDouble(VisionConstants.defaultValue);
  }
  
  public double getAreaOfGoal() {
    RobotContainer.ta = RobotContainer.table.getEntry(VisionConstants.ta);
    return ta.getDouble(VisionConstants.defaultAreaValue);
  }
}
