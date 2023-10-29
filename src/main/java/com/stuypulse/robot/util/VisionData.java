/************************ PROJECT OFSS ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.util;

import edu.wpi.first.math.geometry.Pose3d;

public class VisionData {

    public final Pose3d cameraLocation;
    public final Pose3d robotPose;
    public final double latency;

    public double calculateDistanceToTag(Fiducial tag) {
        return cameraLocation.getTranslation().getDistance(tag.getPose().getTranslation());
    }

    public VisionData(Pose3d cameraLocation, Pose3d robotPose, double latency) {
        this.cameraLocation = cameraLocation;
        this.robotPose = robotPose;
        this.latency = latency;
    }
}
