/************************ PROJECT OFSS ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.subsystems.swerve;

import com.stuypulse.stuylib.math.Vector2D;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Swerve;
import com.stuypulse.robot.subsystems.odometry.AbstractOdometry;
import com.stuypulse.robot.subsystems.odometry.Odometry;
import com.stuypulse.robot.subsystems.swerve.modules.AbstractSwerveModule;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class NoSwerveDrive extends AbstractSwerveDrive {

    private final AbstractSwerveModule[] modules;

    private final SwerveDriveKinematics kinematics;
    private final FieldObject2d[] module2Ds;

    protected NoSwerveDrive(AbstractSwerveModule... modules) {
        this.modules = modules;

        kinematics = new SwerveDriveKinematics(getModuleOffsets());
        module2Ds = new FieldObject2d[modules.length];
    }

    public void initFieldObjects(Field2d field) {
        for (int i = 0; i < modules.length; i++)
            module2Ds[i] = field.getObject(modules[i].getID() + "-2d");
    }

    public Translation2d[] getModuleOffsets() {
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

    public SwerveModuleState filterModuleState(SwerveModuleState state) {
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
        return new Rotation2d();
    }

    public Rotation2d getGyroPitch() {
        return new Rotation2d();
    }

    public Rotation2d getGyroRoll() {
        return new Rotation2d();
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

    private int x;

    @Override
    public void periodic() {
        x += 1;
        SmartDashboard.putString("YOU'RE DONE", "UP" + x);
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
}
