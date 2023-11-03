/************************ PROJECT OFSS ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.commands.swerve;

import java.util.function.Supplier;

import com.stuypulse.robot.constants.Settings.Alignment;
import static com.stuypulse.robot.constants.Settings.Alignment.*;

import com.stuypulse.robot.subsystems.odometry.AbstractOdometry;
import com.stuypulse.robot.subsystems.swerve.AbstractSwerveDrive;
import com.stuypulse.robot.util.HolonomicController;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounceRC;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SwerveDriveToPose extends CommandBase{
    private final AbstractSwerveDrive swerve;
    private final Supplier<Pose2d> targetPoses;

    // Holonomic control
    private final HolonomicController controller;
    private final BStream aligned;

    private final FieldObject2d targetPose2d;

    public SwerveDriveToPose(Supplier<Pose2d> targetPoses){
        this.swerve = AbstractSwerveDrive.getInstance();
        this.targetPoses = targetPoses;

        controller = new HolonomicController(
            new PIDController(Translation.P,Translation.I,Translation.D),
            new PIDController(Translation.P, Translation.I, Translation.D),
            new AnglePIDController(Rotation.P, Rotation.I, Rotation.D));

        SmartDashboard.putData("Alignment/Controller", controller);

        aligned = BStream.create(this::isAligned).filtered(new BDebounceRC.Rising(Alignment.DEBOUNCE_TIME));

        targetPose2d = AbstractOdometry.getInstance().getField().getObject("Target Pose");
        addRequirements(swerve);
    }

    private boolean isAligned() {
        return controller.isDone(0.025, 0.025, 2);
    }

    @Override
    public void execute() {
        Pose2d currentPose = AbstractOdometry.getInstance().getPose();
        Pose2d targetPose = targetPoses.get();

        targetPose2d.setPose(targetPose);

        controller.update(targetPose, currentPose);
        swerve.setChassisSpeeds(controller.getOutput());
    }

    @Override
    public boolean isFinished(){
        return aligned.get();
    }

    public void end(boolean interupted) {
        swerve.stop();
        targetPose2d.setPose(Double.NaN, Double.NaN, new Rotation2d(Double.NaN));
    }
}