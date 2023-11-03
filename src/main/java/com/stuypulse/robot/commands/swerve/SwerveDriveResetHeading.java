package com.stuypulse.robot.commands.swerve;

import com.stuypulse.robot.subsystems.odometry.AbstractOdometry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class SwerveDriveResetHeading extends InstantCommand {
    
    private final AbstractOdometry odometry;
    private final Pose2d pose;

    public SwerveDriveResetHeading(Pose2d pose) {
        this.odometry = AbstractOdometry.getInstance();
        this.pose = pose;
    }

    @Override
    public void initialize() {
        odometry.reset(pose);
    }
}
