package frc.robot.dpad;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class DpadJoystick {

    public class DpadJoystickRight extends Trigger {
        public Joystick joy;

        public DpadJoystickRight(Joystick joy) {
            this.joy = joy;
        }

        @Override
        public boolean get() {
            // This returns whether the trigger is active
            return (this.joy.getPOV() >= 45 && this.joy.getPOV() <= 135);
        }

    }

    public class DpadJoystickLeft extends Trigger {
        public Joystick joy;

        public DpadJoystickLeft(Joystick joy) {
            this.joy = joy;
        }

        @Override
        public boolean get() {
            // This returns whether the trigger is active
            return (this.joy.getPOV() >= 225 && this.joy.getPOV() <= 315);
        }

    }

    public class DpadJoystickUp extends Trigger {
        public Joystick joy;

        public DpadJoystickUp(Joystick joy) {
            this.joy = joy;
        }

        @Override
        public boolean get() {
            // This returns whether the trigger is active
            return (this.joy.getPOV() >= 315 && this.joy.getPOV() < 360)
                    || (this.joy.getPOV() >= 0 && this.joy.getPOV() <= 45);
        }

    }

    public class DpadJoystickDown extends Trigger {
        public Joystick joy;

        public DpadJoystickDown(Joystick joy) {
            this.joy = joy;
        }

        @Override
        public boolean get() {
            // This returns whether the trigger is active
            return (this.joy.getPOV() >= 135 && this.joy.getPOV() <= 225);
        }

    }

}
