/************************ PROJECT OFSS ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.subsystems.swerve;

import com.stuypulse.stuylib.math.Vector2D;

import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Robot;
import com.stuypulse.robot.constants.Settings.Swerve.Modules.BackLeft;
import com.stuypulse.robot.constants.Settings.Swerve.Modules.BackRight;
import com.stuypulse.robot.constants.Settings.Swerve.Modules.FrontLeft;
import com.stuypulse.robot.constants.Settings.Swerve.Modules.FrontRight;
import com.stuypulse.robot.subsystems.swerve.modules.AbstractSwerveModule;
import com.stuypulse.robot.subsystems.swerve.modules.SPSwerveModule;
import com.stuypulse.robot.subsystems.swerve.modules.SimModule;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class AbstractSwerveDrive extends SubsystemBase {

    private static final AbstractSwerveDrive instance;

    static {
        if (RobotBase.isReal() && Settings.ROBOT == Robot.REAL) {
            instance =
                    new SwerveDrive(
                            new SPSwerveModule(
                                    FrontRight.ID,
                                    Ports.Swerve.FrontRight.TURN_MOTOR,
                                    0,
                                    Ports.Swerve.FrontRight.DRIVE_MOTOR,
                                    FrontRight.MODULE_LOCATION,
                                    FrontRight.WHEEL_ROTATION_OFFSET),
                            new SPSwerveModule(
                                    FrontRight.ID,
                                    Ports.Swerve.FrontRight.TURN_MOTOR,
                                    0,
                                    Ports.Swerve.FrontRight.DRIVE_MOTOR,
                                    FrontRight.MODULE_LOCATION,
                                    FrontRight.WHEEL_ROTATION_OFFSET),
                            new SPSwerveModule(
                                    FrontRight.ID,
                                    Ports.Swerve.FrontRight.TURN_MOTOR,
                                    0,
                                    Ports.Swerve.FrontRight.DRIVE_MOTOR,
                                    FrontRight.MODULE_LOCATION,
                                    FrontRight.WHEEL_ROTATION_OFFSET),
                            new SPSwerveModule(
                                    FrontRight.ID,
                                    Ports.Swerve.FrontRight.TURN_MOTOR,
                                    0,
                                    Ports.Swerve.FrontRight.DRIVE_MOTOR,
                                    FrontRight.MODULE_LOCATION,
                                    FrontRight.WHEEL_ROTATION_OFFSET));
        } else if (RobotBase.isReal() && Settings.ROBOT == Robot.TESTBOARD) {
            instance = new NoSwerveDrive(
                new SimModule(FrontRight.ID, FrontRight.MODULE_LOCATION),
                new SimModule(FrontLeft.ID, FrontLeft.MODULE_LOCATION),
                new SimModule(BackLeft.ID, BackLeft.MODULE_LOCATION),
                new SimModule(BackRight.ID, BackRight.MODULE_LOCATION));
        } else {
            instance =
                    new SwerveDrive(
                            new SimModule(FrontRight.ID, FrontRight.MODULE_LOCATION),
                            new SimModule(FrontLeft.ID, FrontLeft.MODULE_LOCATION),
                            new SimModule(BackLeft.ID, BackLeft.MODULE_LOCATION),
                            new SimModule(BackRight.ID, BackRight.MODULE_LOCATION));
        }
    }

    public static AbstractSwerveDrive getInstance() {
        return instance;
    }

    protected AbstractSwerveDrive(AbstractSwerveModule... modules) {}

    public abstract void initFieldObjects(Field2d field);

    public abstract Translation2d[] getModuleOffsets();

    public abstract SwerveModulePosition[] getModulePositions();

    public abstract SwerveModuleState[] getModuleStates();

    public abstract ChassisSpeeds getChassisSpeeds();

    public abstract void drive(Vector2D velocity, double omega);

    public abstract void setChassisSpeeds(ChassisSpeeds robotSpeed);

    public abstract SwerveModuleState filterModuleState(SwerveModuleState state);

    public abstract void setModuleStates(SwerveModuleState... states);

    public abstract void stop();

    public abstract Rotation2d getGyroYaw();

    public abstract Rotation2d getGyroPitch();

    public abstract Rotation2d getGyroRoll();

    public abstract SwerveDriveKinematics getKinematics();

    public abstract void setXMode();
}
