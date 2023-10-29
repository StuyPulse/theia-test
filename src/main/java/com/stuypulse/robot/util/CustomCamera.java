package com.stuypulse.robot.util;

import com.stuypulse.robot.constants.Field;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArraySubscriber;
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

    private final DoubleArraySubscriber robotPoseSub;
    private final IntegerSubscriber fpsSub;

    private final Pose3d cameraPose;

    private double[] rawData;
    
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

        var outputTable = table.getSubTable("output");
        robotPoseSub = outputTable.getDoubleArrayTopic("robot_pose")
                .subscribe(new double[] {}, PubSubOption.keepDuplicates(true), PubSubOption.sendAll(true));

        fpsSub = outputTable.getIntegerTopic("fps").subscribe(0);

        this.cameraPose = cameraPose;
    }

    public String getCameraName() {
        return cameraName;
    }

    public void updateData() {
        rawData = robotPoseSub.get();
    }

    public boolean hasData() {
        return rawData.length > 0;
    }

    private Pose3d getRobotPose() {
        return new Pose3d(
            new Translation3d(
                rawData[0], 
                rawData[1], 
                rawData[2]
            ),
            new Rotation3d(
                Units.degreesToRadians(rawData[3]), 
                Units.degreesToRadians(rawData[4]), 
                Units.degreesToRadians(rawData[5])
            )
        );
    }

    private double getLatency() {
        return Units.millisecondsToSeconds(rawData[6]);
    }

    public VisionData getVisionData() {
        return new VisionData(cameraPose, getRobotPose(), getLatency());
    }

    public long getFPS() {
        return fpsSub.get();
    }
}
