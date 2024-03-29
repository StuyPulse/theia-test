/************************ PROJECT OFSS ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.util;

import com.stuypulse.robot.constants.Field;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.IntegerArraySubscriber;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;

public class CustomCamera {

    private final String cameraName;

    // Default Values
    private final int camera_id = 0;
    private final int camera_resolution_width = 1600;
    private final int camera_resolution_height = 1200;
    private final int camera_auto_exposure = 1;
    private final int camera_exposure = 1000;
    private final double camera_gain = 0.0;
    private final double camera_brightness = 0.0;

    private final DoubleArraySubscriber robotPoseSub;
    private final DoubleSubscriber latencySub;
    private final IntegerSubscriber fpsSub;
    private final DoubleArraySubscriber tvecsSub;
    private final IntegerArraySubscriber idSub;

    private final Pose3d cameraPose;

    private double[] rawPoseData;
    private double latency;
    private double[] rawTvecsData;
    private long[] rawIdData;

    public CustomCamera(String cameraName, Pose3d cameraPose) {

        this.cameraName = cameraName;

        NetworkTable table = NetworkTableInstance.getDefault().getTable(cameraName);

        NetworkTable configTable = table.getSubTable("config");
        configTable.getIntegerTopic("camera_id").publish().set(camera_id);
        configTable.getIntegerTopic("camera_resolution_width").publish().set(camera_resolution_width);
        configTable.getIntegerTopic("camera_resolution_height").publish().set(camera_resolution_height);
        configTable.getIntegerTopic("camera_auto_exposure").publish().set(camera_auto_exposure);
        configTable.getIntegerTopic("camera_exposure").publish().set(camera_exposure);
        configTable.getDoubleTopic("camera_gain").publish().set(camera_gain);
        configTable.getDoubleTopic("camera_brightness").publish().set(camera_brightness);
        configTable.getDoubleTopic("fiducial_size").publish().set(Field.FIDUCIAL_SIZE);
        configTable.getDoubleArrayTopic("fiducial_layout").publish().set(Field.getTagLayout(Field.TAGS));

        NetworkTable outputTable = table.getSubTable("output");
        robotPoseSub = outputTable.getDoubleArrayTopic("robot_pose")
            .subscribe(
                new double[] {},
                PubSubOption.keepDuplicates(true),
                PubSubOption.sendAll(true));
        latencySub = outputTable.getDoubleTopic("latency").subscribe(0);
        fpsSub = outputTable.getIntegerTopic("fps").subscribe(0);
        tvecsSub = outputTable.getDoubleArrayTopic("tvecs")
            .subscribe(
                new double[] {},
                PubSubOption.keepDuplicates(true),
                PubSubOption.sendAll(true));
        idSub = outputTable.getIntegerArrayTopic("ids").subscribe(new long[] {});

        this.cameraPose = cameraPose;
    }

    public String getCameraName() {
        return cameraName;
    }

    public void updateData() {
        rawPoseData = robotPoseSub.get();
        latency = latencySub.get();
        rawTvecsData = tvecsSub.get();
        rawIdData = idSub.get();
    }

    public boolean hasData() {
        return rawPoseData.length > 0 && 
               rawTvecsData.length > 0 && 
               rawIdData.length > 0;
    }

    private Pose3d getRobotPose() {
        return new Pose3d(
                new Translation3d(rawPoseData[0], rawPoseData[1], rawPoseData[2]),
                new Rotation3d(
                        Units.degreesToRadians(rawPoseData[3]),
                        Units.degreesToRadians(rawPoseData[4]),
                        Units.degreesToRadians(rawPoseData[5])));
    }
    
    private Translation3d[] getTvecs() {
        Translation3d[] tvecs = new Translation3d[rawTvecsData.length / 3];
        for (int i = 0; i < rawTvecsData.length; i += 3)
            tvecs[i / 3] = new Translation3d(rawTvecsData[i], rawTvecsData[i + 1], rawTvecsData[i + 2]);
        return tvecs;
    }

    private double getLatency() {
        return Units.millisecondsToSeconds(latency);
    }

    public VisionData getVisionData() {
        return new VisionData(rawIdData, getTvecs(), cameraPose, getRobotPose(), getLatency());
    }

    public long getFPS() {
        return fpsSub.get();
    }
}
