package com.stuypulse.robot.constants;

/** This file contains the different ports of motors, solenoids and sensors */
public interface Ports {
    public interface Gamepad {
        int DRIVER = 0;
        int OPERATOR = 1;
        int DEBUGGER = 2;
    }

    public interface Swerve {
        public interface FrontLeft {
            int DRIVE_MOTOR = 11;
            int TURN_MOTOR = 12;
            int TURN_ENCODER = 1;
        }

        public interface FrontRight {
            int DRIVE_MOTOR = 13;
            int TURN_MOTOR = 14;
            int TURN_ENCODER = 2;
        }

        public interface BackLeft {
            int DRIVE_MOTOR = 15;
            int TURN_MOTOR = 16;
            int TURN_ENCODER = 3;
        }

        public interface BackRight {
            int DRIVE_MOTOR = 17;
            int TURN_MOTOR = 18;
            int TURN_ENCODER = 4;
        }
    }
}
