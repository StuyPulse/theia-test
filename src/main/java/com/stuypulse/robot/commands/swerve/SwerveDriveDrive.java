/************************ PROJECT OFSS ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.commands.swerve;

import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.math.SLMath;
import com.stuypulse.stuylib.streams.IStream;
import com.stuypulse.stuylib.streams.filters.LowPassFilter;
import com.stuypulse.stuylib.streams.vectors.VStream;
import com.stuypulse.stuylib.streams.vectors.filters.VDeadZone;
import com.stuypulse.stuylib.streams.vectors.filters.VLowPassFilter;
import com.stuypulse.stuylib.streams.vectors.filters.VRateLimit;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SwerveDriveDrive extends CommandBase {

    private final SwerveDrive swerve;

    private VStream velocity;
    private IStream omega;

    public SwerveDriveDrive(Gamepad driver) {
        this.swerve = SwerveDrive.getInstance();

        velocity =
                VStream.create(driver::getLeftStick)
                        .filtered(
                                new VDeadZone(Settings.Driver.DEADBAND),
                                x -> x.clamp(1),
                                x -> Settings.vpow(x, Settings.Driver.POWER.get()),
                                x -> x.mul(Settings.Driver.MAX_SPEED.get()),
                                new VRateLimit(Settings.Driver.MAX_ACCELERATION),
                                new VLowPassFilter(Settings.Driver.RC));

        omega =
                IStream.create(driver::getRightX)
                        .filtered(
                                x -> SLMath.deadband(x, Settings.Driver.DEADBAND.get()),
                                x -> SLMath.spow(x, Settings.Driver.POWER.get()),
                                x -> x * Settings.Driver.MAX_TURNING.get(),
                                new LowPassFilter(Settings.Driver.RC));

        addRequirements(swerve);
    }

    @Override
    public void execute() {
        swerve.drive(velocity.get(), omega.get());

        SmartDashboard.putNumber("Driver/Input/X Velocity", velocity.get().x);
        SmartDashboard.putNumber("Driver/Input/Y Velocity", velocity.get().y);
        SmartDashboard.putNumber("Driver/Input/Omega", omega.get());
    }
}
