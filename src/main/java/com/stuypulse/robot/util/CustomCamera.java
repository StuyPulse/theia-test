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
    private final int camera_auto_exposure = 0;
    private final int camera_exposure = 0;
    private final double camera_gain = 0.0;
    private final double camera_brightness = 0.0;

    private final IntegerSubscriber fpsSub;

    private final IntegerSubscriber idSub;
    private final DoubleSubscriber robot_x_sub;
    private final DoubleSubscriber robot_y_sub;
    private final DoubleSubscriber robot_z_sub;
    private final DoubleSubscriber robot_roll_sub;
    private final DoubleSubscriber robot_pitch_sub;
    private final DoubleSubscriber robot_yaw_sub;
    private final DoubleSubscriber latency_sub;

    private final Pose3d cameraPose;

    private double robot_x;
    private double robot_y;
    private double robot_z;
    private double robot_roll;
    private double robot_pitch;
    private double robot_yaw;
    private double latency;
    private long id;

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
        
        fpsSub = outputTable.getIntegerTopic("fps").subscribe(0);

        robot_x_sub = outputTable.getDoubleTopic("robot_pose_x").subscribe(0);
        robot_y_sub = outputTable.getDoubleTopic("robot_pose_y").subscribe(0);
        robot_z_sub = outputTable.getDoubleTopic("robot_pose_z").subscribe(0);
        robot_roll_sub = outputTable.getDoubleTopic("robot_pose_roll").subscribe(0);
        robot_pitch_sub = outputTable.getDoubleTopic("robot_pose_pitch").subscribe(0);
        robot_yaw_sub = outputTable.getDoubleTopic("robot_pose_yaw").subscribe(0);

        latency_sub = outputTable.getDoubleTopic("latency").subscribe(-1);
        idSub = outputTable.getIntegerTopic("id").subscribe(-1);

        this.cameraPose = cameraPose;
    }

    public String getCameraName() {
        return cameraName;
    }

    public void updateData() {
        robot_x = robot_x_sub.get();
        robot_y = robot_y_sub.get();
        robot_z = robot_z_sub.get();
        robot_roll = robot_roll_sub.get();
        robot_pitch = robot_pitch_sub.get();
        robot_yaw = robot_yaw_sub.get();
        latency = latency_sub.get();
        id = idSub.get();
    }

    public boolean hasData() {
        return id != -1;
    }

    private Pose3d getRobotPose() {
        return new Pose3d(
            new Translation3d(robot_x, robot_y, robot_z),
            new Rotation3d(Units.degreesToRadians(robot_roll), Units.degreesToRadians(robot_pitch), Units.degreesToRadians(robot_yaw))
        );
    }
    
    private Translation3d[] getTvecs() {
        return null;
    }

    private double getLatency() {
        return Units.millisecondsToSeconds(latency);
    }

    public VisionData getVisionData() {
        return new VisionData(id, getTvecs(), cameraPose, getRobotPose(), getLatency());
    }

    public long getFPS() {
        return fpsSub.get();
    }
}
