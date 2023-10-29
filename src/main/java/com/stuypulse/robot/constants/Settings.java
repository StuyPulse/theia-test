/************************ PROJECT OFSS ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.constants;

import com.stuypulse.stuylib.math.Vector2D;
import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

import com.pathplanner.lib.util.PIDConstants;

/*-
 * File containing tunable settings for every subsystem on the robot.
 *
 * We use StuyLib's SmartNumber / SmartBoolean in order to have tunable
 * values that we can edit on Shuffleboard.
 */
public interface Settings {

    double DT = 0.02;

    public static Vector2D vpow(Vector2D vec, double power) {
        return vec.mul(Math.pow(vec.magnitude(), power - 1));
    }

    public interface Driver {
        SmartNumber DEADBAND = new SmartNumber("Driver/Deadband", 0.0);
        SmartNumber POWER = new SmartNumber("Driver/Power", 2);
        SmartNumber MAX_SPEED = new SmartNumber("Driver/Max Speed", Swerve.MAX_MODULE_SPEED.get());
        SmartNumber MAX_ACCELERATION =
                new SmartNumber("Driver/Max Acceleration", MAX_SPEED.get() * 3);
        SmartNumber MAX_TURNING = new SmartNumber("Driver/Max Turning", 6.0);
        SmartNumber RC = new SmartNumber("Driver/RC", 0.05);
    }

    public interface Swerve {
        double WIDTH = Units.inchesToMeters(25);
        double LENGTH = Units.inchesToMeters(25);

        SmartNumber MAX_MODULE_SPEED = new SmartNumber("Swerve/Max Module Speed", 5.0);
        SmartNumber MAX_TURNING = new SmartNumber("Swerve/Max Turn Velocity", Math.PI * 2);
        SmartNumber MODULE_VELOCITY_DEADBAND =
                new SmartNumber("Swerve/Module Velocity Deadband", 0.02);

        public interface Motion {
            PIDConstants XY = new PIDConstants(1, 0, 0.0);
            PIDConstants THETA = new PIDConstants(10, 0, 0.0);
        }

        public interface Encoder {
            public interface Drive {
                double WHEEL_DIAMETER = Units.inchesToMeters(3);
                double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
                double GEAR_RATIO = 1.0 / 4.71;

                double POSITION_CONVERSION = WHEEL_CIRCUMFERENCE * GEAR_RATIO;
                double VELOCITY_CONVERSION = POSITION_CONVERSION / 60.0;
            }

            public interface Turn {
                double POSITION_CONVERSION = 1;
                double VELOCITY_CONVERSION = POSITION_CONVERSION / 60.0;

                double MIN_PID_INPUT = 0;
                double MAX_PID_INPUT = POSITION_CONVERSION;
            }
        }

        public interface Turn {
            SmartNumber kP = new SmartNumber("Swerve/Turn/kP", 3.5);
            SmartNumber kI = new SmartNumber("Swerve/Turn/kI", 0.0);
            SmartNumber kD = new SmartNumber("Swerve/Turn/kD", 0.0);

            SmartNumber kS = new SmartNumber("Swerve/Turn/kS", 0.0);
            SmartNumber kV = new SmartNumber("Swerve/Turn/kV", 0.25);
            SmartNumber kA = new SmartNumber("Swerve/Turn/kA", 0.008);
        }

        public interface Drive {
            SmartNumber kP = new SmartNumber("Swerve/Drive/kP", 0.8);
            SmartNumber kI = new SmartNumber("Swerve/Drive/kI", 0.0);
            SmartNumber kD = new SmartNumber("Swerve/Drive/kD", 0.0);

            SmartNumber kS = new SmartNumber("Swerve/Drive/kS", 0.22304);
            SmartNumber kV = new SmartNumber("Swerve/Drive/kV", 2.4899);
            SmartNumber kA = new SmartNumber("Swerve/Drive/kA", 0.41763);
        }

        public interface Modules {
            public interface FrontRight {
                String ID = "Front Right";
                Rotation2d WHEEL_ROTATION_OFFSET =
                        Rotation2d.fromDegrees(0).plus(Rotation2d.fromDegrees(0));
                Translation2d MODULE_LOCATION = new Translation2d(WIDTH * +0.5, LENGTH * -0.5);
            }

            public interface FrontLeft {
                String ID = "Front Left";
                Rotation2d WHEEL_ROTATION_OFFSET =
                        Rotation2d.fromDegrees(0).plus(Rotation2d.fromDegrees(270));
                Translation2d MODULE_LOCATION = new Translation2d(WIDTH * +0.5, LENGTH * +0.5);
            }

            public interface BackLeft {
                String ID = "Back Left";
                Rotation2d WHEEL_ROTATION_OFFSET =
                        Rotation2d.fromDegrees(0).plus(Rotation2d.fromDegrees(180));
                Translation2d MODULE_LOCATION = new Translation2d(WIDTH * -0.5, LENGTH * +0.5);
            }

            public interface BackRight {
                String ID = "Back Right";
                Rotation2d WHEEL_ROTATION_OFFSET =
                        Rotation2d.fromDegrees(0).plus(Rotation2d.fromDegrees(90));
                Translation2d MODULE_LOCATION = new Translation2d(WIDTH * -0.5, LENGTH * -0.5);
            }
        }
    }
}
