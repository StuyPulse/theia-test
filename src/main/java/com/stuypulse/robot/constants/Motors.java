/************************ PROJECT OFSS ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

/*-
 * File containing all of the configurations that different motors require.
 *
 * Such configurations include:
 *  - If it is Inverted
 *  - The Idle Mode of the Motor
 *  - The Current Limit
 *  - The Open Loop Ramp Rate
 */
public interface Motors {

    int kDisableStatusFrame = 65535;

    public static void disableStatusFrames(CANSparkMax motor, int... ids) {
        for (int id : ids)
            motor.setPeriodicFramePeriod(PeriodicFrame.fromId(id), kDisableStatusFrame);
    }

    public interface Swerve {
        CANSparkMaxConfig DRIVE =
                new CANSparkMaxConfig(false, IdleMode.kBrake, new CurrentLimit(60, 80), 0);
        CANSparkMaxConfig TURN =
                new CANSparkMaxConfig(false, IdleMode.kBrake, new CurrentLimit(20, 40), 0);
    }

    public static class CANSparkMaxConfig {
        public final boolean INVERTED;
        public final IdleMode IDLE_MODE;
        public final CurrentLimit CURRENT_LIMIT;
        public final double OPEN_LOOP_RAMP_RATE;

        public CANSparkMaxConfig(
                boolean inverted,
                IdleMode idleMode,
                CurrentLimit currentLimitAmps,
                double openLoopRampRate) {
            this.INVERTED = inverted;
            this.IDLE_MODE = idleMode;
            this.CURRENT_LIMIT = currentLimitAmps;
            this.OPEN_LOOP_RAMP_RATE = openLoopRampRate;
        }

        public CANSparkMaxConfig(
                boolean inverted,
                IdleMode idleMode,
                int currentLimitAmps,
                double openLoopRampRate) {
            this(inverted, idleMode, new CurrentLimit(currentLimitAmps), openLoopRampRate);
        }

        public CANSparkMaxConfig(boolean inverted, IdleMode idleMode, int currentLimitAmps) {
            this(inverted, idleMode, currentLimitAmps, 0.0);
        }

        public CANSparkMaxConfig(boolean inverted, IdleMode idleMode) {
            this(inverted, idleMode, 80);
        }

        public void configure(CANSparkMax motor) {
            motor.setInverted(INVERTED);
            motor.setIdleMode(IDLE_MODE);
            motor.setSmartCurrentLimit(
                    CURRENT_LIMIT.STALL_LIMIT_AMPS, CURRENT_LIMIT.FREE_LIMIT_AMPS);
            motor.setOpenLoopRampRate(OPEN_LOOP_RAMP_RATE);
            motor.burnFlash();
        }
    }

    public static class CurrentLimit {
        public final int STALL_LIMIT_AMPS;
        public final int FREE_LIMIT_AMPS;

        public CurrentLimit(int currentLimitAmps, int freeLimitAmps) {
            STALL_LIMIT_AMPS = currentLimitAmps;
            FREE_LIMIT_AMPS = freeLimitAmps;
        }

        public CurrentLimit(int currentLimitAmps) {
            this(currentLimitAmps, 100);
        }

        public CurrentLimit() {
            this(80, 100);
        }
    }
}
