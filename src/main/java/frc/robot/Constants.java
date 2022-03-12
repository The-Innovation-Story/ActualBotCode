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
        public static final int FL_ID = 21; // 11
        public static final int BL_ID = 22; // 12
        public static final int FR_ID = 11; // 21
        public static final int BR_ID = 12; // 22
        public static final double kWheelRadius = Units.inchesToMeters(6);
        public static final double kMaxSpeed = 0.5; // multiplier for distance movement
        public static final double kMaxAngularSpeed = 0.5; // multiplier for angular movement

        public static final double kTrackWidth = 0.61; // meters
        public static final double kP = 5;
        public static final double kI = 0.2;
        public static final double kD = 1.2;
        public static final double kPTurn = 5;
        public static final double kITurn = 0.2;
        public static final double kDTurn = 1.2;
        public static final double rpm_to_ms_wheel_converter = (Math.PI / 30) * Units.inchesToMeters(3);
        public static final double kRiseLimiter = 3;
        public static final double kMaxVelocity = 0.5;
        public static final double kMaxAcceleration = 0.5;
        public static final double kMaxVelocityTurning = 0.5;
        public static final double kMaxAccelerationTurning = 0.5;
        public static final double kMinimumAutonomousDriveSpeed = -0.3;
        public static final double kMaximumAutonomousDriveSpeed = 0.3;
        public static final double kMinimumAutonomousTurnSpeed = -0.3;
        public static final double kMaximumAutonomousTurnSpeed = 0.3;
    }

    public final static class OIConstants {

        public static final int kDriverJoystickPort = 0;
        public static final int kClimberJoystickPort = 1;
        public static final int kJoyDTurnAxis = 0; // 4: Matunga
        public static final int kJoyDSpeedAxis = 1;
        public static final int feeder_X_ButtonNumber = 1;
        public static final double feederDebouncePeriod = 0.5;
        public static final int intakeForward_Y_ButtonNumber = 3;
        public static final int shooter_RB_ButtonNumber = 5;
        public static final int turn_LB_ButtonNumber = 6;

        public final static class OIJoyC {

            public static final int innerPG_Axis_Two = 2;
            public static final int outerPG_Axis_Three = 3;
            public static final int innerPG_Button_Five = 5;
            public static final int outerPG_Button_Six = 6;
            
            public static final int innerClimber_Axis_One = 1;
            public static final int outerClimber_Axis_Five = 5;

            public static final int innerPGStop_Button_Two = 2;
            public static final int outerPGStop_Button_Three = 3;

        }

    }

    public final static class IntakeConstants {

        public static final int intake_ID = 24;
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

        public static final int feeder_ID = 29;
        public static final double deadband = 0.05;
        public static final double feederSpeed = 0.35;
        public static final int servo_port = 0;
        public static final double initialAngle = 50;
        public static final double positionAngle = 140;
    }

    public final static class ShooterConstants {

        public static final int shooter_ID = 1;
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
        public static final double deadbandVelocity = 500;
        public static double setThisVelocity = 2500;
    }

    public final static class ClimberConstants {

        public static final int falcon_ID = 6;
        public static final double falconSpeed = 0.5;
        public static final double deadband = 0.05;
        public static final double speedMultiplier = 0.9;

        public final static class PGConstants {

            public static final int innerPGConstants = 4;
            public static final int outerPGConstants = 6;
            public static final double speedMultiplier = 0.35;
            public static final double kPInner = 0.097;
            public static final double kIInner = 0;
            public static final double kDInner = 0;
            public static final double kPOuter = 0.07;
            public static final double kIOuter = 0;
            public static final double kDOuter = 0;
            public static final double deadband = 0.05;

        }

        public final static class InnerClimberConstants {

            public static final int in_LC_ID = 31;
            public static final int in_RC_ID = 32;
            public static final double speedMultiplier = 0.9;

        }

        public final static class OuterClimberConstants {

            public static final int out_LC_ID = 5;
            public static final int out_RC_ID = 7;
            public static final double speedMultiplier = 0.9;

        }

    }
}
