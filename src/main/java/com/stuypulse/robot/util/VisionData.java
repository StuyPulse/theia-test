package com.stuypulse.robot.util;

import edu.wpi.first.math.geometry.Pose3d;

public class VisionData {
    
    public final Pose3d cameraLocation;
    public final Pose3d robotPose;
    public final double latency;

    public VisionData(Pose3d cameraLocation, Pose3d robotPose, double latency) {
        this.cameraLocation = cameraLocation;
        this.robotPose = robotPose;
        this.latency = latency;
    }
}
