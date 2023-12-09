/************************ PROJECT OFSS ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.subsystems.swerve.modules;

import static com.stuypulse.robot.constants.Motors.Swerve.*;
import static com.stuypulse.robot.constants.Settings.Swerve.*;

import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.angle.AngleController;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.control.feedforward.MotorFeedforward;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.streams.angles.filters.ARateLimit;

import com.stuypulse.robot.constants.Motors;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;

public class SPSwerveModule extends AbstractSwerveModule {

    private final String ID;
    private final Translation2d moduleLocation; // Location of module relative to robot center
    private SwerveModuleState targetState;

    private final CANSparkMax turnMotor;
    // private final CANCoder turnCANCoder;
    private final DutyCycleEncoder turnCANCoder;

    private final CANSparkMax driveMotor;
    private final RelativeEncoder driveEncoder;

    private final Rotation2d wheelRotationOffset; // Angle offset of wheel from "front" of robot

    private AngleController turnController;
    private Controller driveController;

    public SPSwerveModule(
            String ID,
            int turnCANID,
            int turnCANCoderID,
            int driveCANID,
            Translation2d moduleLocation,
            Rotation2d wheelRotationOffset) {
        this.ID = ID;
        this.moduleLocation = moduleLocation;
        this.targetState = new SwerveModuleState();

        turnMotor = new CANSparkMax(turnCANID, CANSparkMax.MotorType.kBrushless);
        driveMotor = new CANSparkMax(driveCANID, CANSparkMax.MotorType.kBrushless);

        // turnCANCoder = new CANCoder(turnCANCoderID);
        turnCANCoder = new DutyCycleEncoder(turnCANCoderID);
        driveEncoder = driveMotor.getEncoder();

        configureMotors();

        this.wheelRotationOffset = wheelRotationOffset;

        turnController =
                new AnglePIDController(Turn.kP, Turn.kI, Turn.kD)
                        .setSetpointFilter(new ARateLimit(MAX_TURNING.getAsDouble()));
        driveController =
                new PIDController(Drive.kP, Drive.kI, Drive.kD)
                        .add(new MotorFeedforward(Drive.kS, Drive.kV, Drive.kA).velocity());
    }

    private void configureMotors() {
        // CANCoderConfiguration turnConfig = new CANCoderConfiguration();
        // turnConfig.magnetOffsetDegrees = 0;
        // turnConfig.sensorDirection = true;
        // turnCANCoder.configAllSettings(turnConfig);

        Motors.disableStatusFrames(turnMotor, 3, 4, 5, 6);
        TURN.configure(turnMotor);

        driveEncoder.setPositionConversionFactor(Encoder.Drive.POSITION_CONVERSION);
        driveEncoder.setVelocityConversionFactor(Encoder.Drive.VELOCITY_CONVERSION);
        
        Motors.disableStatusFrames(driveMotor, 3, 4, 5, 6);
        DRIVE.configure(driveMotor);
    }

    @Override
    public String getID() {
        return ID;
    }

    @Override
    public Translation2d getModuleLocation() {
        return moduleLocation;
    }

    @Override
    public Rotation2d getWheelRotationOffset() {
        return wheelRotationOffset;
    }

    private double getVelocity() {
        return driveEncoder.getVelocity();
    }

    private double getPosition() {
        return driveEncoder.getPosition();
    }

    private Rotation2d getAbsolutePosition() {
        return new Rotation2d(MathUtil.interpolate(-Math.PI, +Math.PI, turnCANCoder.getAbsolutePosition()));
    }

    private Rotation2d getAngle() {
        return getAbsolutePosition().minus(wheelRotationOffset);
    }

    @Override
    public SwerveModuleState getState() {
        return new SwerveModuleState(getVelocity(), getAngle());
    }

    @Override
    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(getPosition(), getAngle());
    }

    @Override
    public void setTargetState(SwerveModuleState swerveModuleState) {
        // targetState = swerveModuleState;
        targetState = SwerveModuleState.optimize(swerveModuleState, getAngle());
    }

    @Override
    public void periodic() {

        turnMotor.setVoltage(
                turnController.update(
                        Angle.fromRotation2d(targetState.angle), 
                        Angle.fromRotation2d(getAngle())));
        driveMotor.setVoltage(
                driveController.update(targetState.speedMetersPerSecond, getVelocity()));

        SmartDashboard.putNumber("Swerve/" + ID + "/Raw Angle", Units.rotationsToDegrees(turnCANCoder.getAbsolutePosition()));
        SmartDashboard.putNumber("Swerve/" + ID + "/Angle", getAngle().getDegrees());
        SmartDashboard.putNumber("Swerve/" + ID + "/Angle Error", turnController.getError().toDegrees());
        SmartDashboard.putNumber("Swerve/" + ID + "/Target Angle", targetState.angle.getDegrees());

        SmartDashboard.putNumber("Swerve/" + ID + "/Turn Motor Voltage", turnController.getOutput());
        SmartDashboard.putNumber("Swerve/" + ID + "/Turn Motor Current", turnMotor.getOutputCurrent());

        SmartDashboard.putNumber("Swerve/" + ID + "/Velocity", getVelocity());
        SmartDashboard.putNumber("Swerve/" + ID + "/Velocity Error", driveController.getError());
        SmartDashboard.putNumber("Swerve/" + ID + "/Target Velocity", targetState.speedMetersPerSecond);

        SmartDashboard.putNumber("Swerve/" + ID + "/Drive Motor Voltage", driveController.getOutput());
        SmartDashboard.putNumber("Swerve/" + ID + "/Drive Motor Current", driveMotor.getOutputCurrent());
    }
}
