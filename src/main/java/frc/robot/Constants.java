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
        public static final int FL_ID = 22; // 22
        public static final int BL_ID = 21; // 21
        public static final int FR_ID = 12; // 12
        public static final int BR_ID = 11; // 11
        public static final double kWheelRadius = Units.inchesToMeters(6);
        public static final double kMaxSpeed = 0.5; // multiplier for distance movement
        public static final double kMaxAngularSpeed = 0.5; // multiplier for angular movement

        public static final double kTrackWidth = 0.61; // meters
        public static final double kP = 0.05;
        public static final double kI = 0.0;
        public static final double kD = 0;
        public static final double kPTurn = 0.05;
        public static final double kITurn = 0.0;
        public static final double kDTurn = 0;
        public static final double rpm_to_ms_wheel_converter = (Math.PI / 30) * Units.inchesToMeters(3);
        public static final double kRiseLimiter = 3;
        public static final double kMaxVelocity = 5;
        public static final double kMaxAcceleration = 1;
        public static final double kMaxVelocityTurning = 5;
        public static final double kMaxAccelerationTurning = 1;
        public static final double kMinimumAutonomousDriveSpeed = -0.3;
        public static final double kMaximumAutonomousDriveSpeed = 0.3;
        public static final double kMinimumAutonomousTurnSpeed = -0.3;
        public static final double kMaximumAutonomousTurnSpeed = 0.3;
    }

    public final static class OIConstants {

        public static final int kDriverJoystickPort = 0;
        public static final int kClimberJoystickPort = 1;
        public static final int kJoyDTurnAxis = 4; // 4: Matunga, 0: DAIS
        public static final int kJoyDSpeedAxis = 1;
        public static final int feeder_X_ButtonNumber = 1;
        public static final double feederDebouncePeriod = 0.5;
        public static final int intakeForward_Y_ButtonNumber = 3;
        public static final int shooter_RB_ButtonNumber = 5;
        public static final int turn_LB_ButtonNumber = 6;

        public final static class JoyC {

            public static final int innerUppingButton = 5;
            public static final int innerDowningButton = 6;
            public static final int pgOneAxis = 1;
            public static final int innerTwoAxis = 2;
            public static final int innerFiveAxis = 5;
            public static final int outerFourAxis = 4;
            public static final int pgOuterRunButton = 3;

        }

    }

    public final static class IntakeConstants {

        public static final int intake_ID = 2;
        public static final double deadband = 0.05;
        public static final double stopSpeed = 0;
        public static final double flowSpeed = 0.5;
    }

    public final static class VisionConstants {

        public static final String limelight = "limelight";
        public static final String tv = "tv";
        public static final String tx = "tx";
        public static final String ty = "ty";
        public static final String ta = "ta";
        public static final double defaultValue = 0;
        public static final double defaultAreaValue = 0;
        public static final double kP = 0;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kMaxVelocityTurning = 0.5;
        public static final double kMaxAccelerationTurning = 0.5;
        public static final double minTurnSpeed = -0.3;
        public static final double maxTurnSpeed = 0.3;

    }

    public final static class FeederConstants {

        public static final int feeder_ID = 1;
        public static final double deadband = 0.05;
        public static final double feederSpeed = 0.35;
    }

    public final static class ShooterConstants {

        public static final int shooter_ID = 5;
        public static final double deadband = 0.05;
        public static final double shooterSpeed = 0.35;
        public static final int neoCountsPerRevolution = 42;
        public static final double kP = 0.00012;
        public static final double kI = 3e-8;
        public static final double kD = 1.2;
        public static final double kIz = 0;
        public static final double kMinOutput = -1;
        public static final double kFF = 0.00017;
        public static final double kMaxOutput = 1;
        public static final double deadbandVelocity = 1500;
        public static double setThisVelocity = 3000;
    }

    public final static class ClimberConstants {

        public static final int falcon_ID = 6;
        public static final double falconSpeed = 0.5;
        public static final double deadband = 0.05;
        public static final double speedMultiplier = 0.5;

        public final static class InnerHoldings {

            public static final double kP = 0.07;
            public static final double kI = 0;
            public static final double kD = 0;
            public static final double topHolding = 201;
            public static final double bottomHolding = 400;
            public static final double innerUpperPGPositionDifference = 120;
            public static final double innerDownerPGPositionDifference = 120;

        }

    }
}
