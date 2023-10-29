/************************ PROJECT OFSS ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.subsystems.swerve;

import com.stuypulse.stuylib.math.Vector2D;

import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Swerve;
import com.stuypulse.robot.constants.Settings.Swerve.Modules.BackLeft;
import com.stuypulse.robot.constants.Settings.Swerve.Modules.BackRight;
import com.stuypulse.robot.constants.Settings.Swerve.Modules.FrontLeft;
import com.stuypulse.robot.constants.Settings.Swerve.Modules.FrontRight;
import com.stuypulse.robot.subsystems.odometry.AbstractOdometry;
import com.stuypulse.robot.subsystems.odometry.Odometry;
import com.stuypulse.robot.subsystems.swerve.modules.AbstractSwerveModule;
import com.stuypulse.robot.subsystems.swerve.modules.SPSwerveModule;
import com.stuypulse.robot.subsystems.swerve.modules.SimModule;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.kauailabs.navx.frc.AHRS;

public class SwerveDrive extends SubsystemBase {

    private static final SwerveDrive instance;

    static {
        if (RobotBase.isReal()) {
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
        } else {
            instance =
                    new SwerveDrive(
                            new SimModule(FrontRight.ID, FrontRight.MODULE_LOCATION),
                            new SimModule(FrontLeft.ID, FrontLeft.MODULE_LOCATION),
                            new SimModule(BackLeft.ID, BackLeft.MODULE_LOCATION),
                            new SimModule(BackRight.ID, BackRight.MODULE_LOCATION));
        }
    }

    public static SwerveDrive getInstance() {
        return instance;
    }

    private final AbstractSwerveModule[] modules;

    private final AHRS gyro;
    private final SwerveDriveKinematics kinematics;
    private final FieldObject2d[] module2Ds;

    protected SwerveDrive(AbstractSwerveModule... modules) {
        this.modules = modules;

        gyro = new AHRS(SPI.Port.kMXP);
        kinematics = new SwerveDriveKinematics(getModuleOffsets());
        module2Ds = new FieldObject2d[modules.length];
    }

    public void initFieldObjects(Field2d field) {
        for (int i = 0; i < modules.length; i++)
            module2Ds[i] = field.getObject(modules[i].getID() + "-2d");
    }

    private Translation2d[] getModuleOffsets() {
        Translation2d[] locations = new Translation2d[modules.length];

        for (int i = 0; i < modules.length; ++i) locations[i] = modules[i].getModuleLocation();

        return locations;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];
        for (int i = 0; i < modules.length; i++) positions[i] = modules[i].getModulePosition();
        return positions;
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[modules.length];
        for (int i = 0; i < modules.length; i++) states[i] = modules[i].getState();
        return states;
    }

    public ChassisSpeeds getChassisSpeeds() {
        return getKinematics().toChassisSpeeds(getModuleStates());
    }

    public void drive(Vector2D velocity, double omega) {
        ChassisSpeeds speeds =
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        velocity.y, -velocity.x, -omega, Odometry.getInstance().getRotation());

        Pose2d robotVel =
                new Pose2d(
                        Settings.DT * speeds.vxMetersPerSecond,
                        Settings.DT * speeds.vyMetersPerSecond,
                        Rotation2d.fromRadians(Settings.DT * speeds.omegaRadiansPerSecond));

        Twist2d twistVel = new Pose2d().log(robotVel);

        setChassisSpeeds(
                new ChassisSpeeds(
                        twistVel.dx / Settings.DT,
                        twistVel.dy / Settings.DT,
                        twistVel.dtheta / Settings.DT));
    }

    public void setChassisSpeeds(ChassisSpeeds robotSpeed) {
        setModuleStates(kinematics.toSwerveModuleStates(robotSpeed));
    }

    private static SwerveModuleState filterModuleState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) > Swerve.MODULE_VELOCITY_DEADBAND.get())
            return state;

        return new SwerveModuleState(0, state.angle);
    }

    public void setModuleStates(SwerveModuleState... states) {
        if (states.length != modules.length)
            throw new IllegalArgumentException(
                    "Number of desired module states does not match number of modules ("
                            + modules.length
                            + ")");

        SwerveDriveKinematics.desaturateWheelSpeeds(states, Swerve.MAX_MODULE_SPEED.get());

        for (int i = 0; i < modules.length; i++)
            modules[i].setTargetState(filterModuleState(states[i]));
    }

    public void stop() {
        setChassisSpeeds(new ChassisSpeeds());
    }

    public Rotation2d getGyroYaw() {
        return gyro.getRotation2d();
    }

    public Rotation2d getGyroPitch() {
        return Rotation2d.fromDegrees(gyro.getPitch());
    }

    public Rotation2d getGyroRoll() {
        return Rotation2d.fromDegrees(gyro.getRoll());
    }

    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }

    public void setXMode() {
        SwerveModuleState[] state = {
            new SwerveModuleState(0, Rotation2d.fromDegrees(135)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(225)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(315)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
        };
        setModuleStates(state);
    }

    @Override
    public void periodic() {
        AbstractOdometry odometry = AbstractOdometry.getInstance();
        Pose2d pose = odometry.getPose();
        Rotation2d angle = odometry.getRotation();

        for (int i = 0; i < modules.length; ++i)
            module2Ds[i].setPose(
                    new Pose2d(
                            pose.getTranslation()
                                    .plus(modules[i].getModuleLocation().rotateBy(angle)),
                            modules[i].getState().angle.plus(angle)));

        SmartDashboard.putNumber("Swerve/Gyro Angle", getGyroYaw().getDegrees());
        SmartDashboard.putNumber("Swerve/Gyro Pitch", getGyroPitch().getDegrees());
        SmartDashboard.putNumber("Swerve/Gyro Roll", getGyroRoll().getDegrees());
    }

    @Override
    public void simulationPeriodic() {
        var speeds = getKinematics().toChassisSpeeds(getModuleStates());
        gyro.setAngleAdjustment(
                gyro.getAngle() - Math.toDegrees(speeds.omegaRadiansPerSecond * Settings.DT));
    }
}
