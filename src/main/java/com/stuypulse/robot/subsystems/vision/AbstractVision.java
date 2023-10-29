package com.stuypulse.robot.subsystems.vision;

import java.util.List;

import com.stuypulse.robot.util.VisionData;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class AbstractVision extends SubsystemBase {
    
    private static final AbstractVision instance;

    static {
        instance = new Vision();
    }

    public static AbstractVision getInstance() {
        return instance;
    }

    protected AbstractVision() {}

    public abstract List<VisionData> getOutput();
}
