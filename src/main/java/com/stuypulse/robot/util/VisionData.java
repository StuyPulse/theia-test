/************************ PROJECT OFSS ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.util;

import com.stuypulse.robot.constants.Field;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;

public class VisionData {

    // public final long[] ids;
    public final long id;
    public final Translation3d[] tvecs;
    public final Pose3d cameraLocation;
    public final Pose3d robotPose;
    public final double latency;

    public double calculateDistanceToTag(Fiducial tag) {
        return cameraLocation.getTranslation().getDistance(tag.getPose().getTranslation());
    }

    private int getPrimaryID() {
        return (int) id;
    }

    public Fiducial getPrimaryTag() {
        return Field.getTag(getPrimaryID());
    }

    public VisionData(long id, Translation3d[] tvecs, Pose3d cameraLocation, Pose3d robotPose, double latency) {
        this.id = id;
        this.tvecs = tvecs;
        this.cameraLocation = cameraLocation;
        this.robotPose = robotPose;
        this.latency = latency;
    }
}
