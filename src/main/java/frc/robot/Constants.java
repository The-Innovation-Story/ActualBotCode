// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public final static class DrivingConstants {

        public static final int neoCountsPerRevolution = 42;
        public static final int FL_ID = 11;
        public static final int BL_ID = 12;
        public static final int FR_ID = 21;
        public static final int BR_ID = 22;
        public static final double kWheelRadius = Units.inchesToMeters(6);
        public static final double kMaxSpeed = 3.0; // meters per second
        public static final double kMaxAngularSpeed = 2 * Math.PI; // one rotation per second

        public static final double kTrackWidth = 0.657; // meters
        public static final double kP = 0;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kA = 0;
        public static final double kV = 0;
        public static final double kS = 0;
        public static final double rpm_to_ms_wheel_converter = (Math.PI / 30) * Units.inchesToMeters(3);
        public static final double kRiseLimiter = 0;
    }

    public final static class OIConstants {

        public static final int kDriverJoystickPort = 0;
        public static final int kClimberJoystickPort = 1;
        public static final int kJoyDTurnAxis = 4;
        public static final int kJoyDSpeedAxis = 1;
        public static final int feeder_X_ButtonNumber = 1;
        public static final double feederDebouncePeriod = 0.5;
        public static final int intakeForward_Y_ButtonNumber = 0;

    }

    public final static class IntakeConstants {

        public static final int intake_ID = 29;
        public static final double deadband = 0.05;
        public static final double stopSpeed = 0;
    }

    public final static class VisionConstants {

        public static final String limelight = "limelight";
        public static final String tv = "tv";
        public static final String tx = "tx";
        public static final String ty = "ty";
        public static final String ta = "ta";

    }

    public final static class FeederConstants {

        public static final int feeder_ID = 25;
        public static final double deadband = 0.05;
        public static final double feederSpeed = 0.35;
    }

    public final static class ShooterConstants {

        public static final int shooter_ID = 25;
        public static final double deadband = 0.05;
        public static final double shooterSpeed = 0.35;
        public static final int neoCountsPerRevolution = 0;
        public static final double kP = 0;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kIz = 0;
        public static final double kMinOutput = 0;
        public static final double kFF = 0;
        public static final double kMaxOutput = 0;
        public static final double deadbandVelocity = 0;
        public static final double setThisVelocity = 0;
    }
}
