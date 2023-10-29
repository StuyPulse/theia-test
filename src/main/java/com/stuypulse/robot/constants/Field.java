/************************ PROJECT OFSS ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.constants;

import com.stuypulse.robot.util.Fiducial;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public interface Field {

    public static final double FIDUCIAL_SIZE = 0.15558;

    Fiducial TAGS[] = {
        new Fiducial(
                0,
                new Pose3d(
                        new Translation3d(0, 4, 0),
                        new Rotation3d(
                                Units.degreesToRadians(0),
                                Units.degreesToRadians(0),
                                Units.degreesToRadians(180)))),
        new Fiducial(
                1,
                new Pose3d(
                        new Translation3d(1, 4, 0),
                        new Rotation3d(
                                Units.degreesToRadians(0),
                                Units.degreesToRadians(0),
                                Units.degreesToRadians(180)))),
    };

    public static boolean isValidTag(int id) {
        return id >= 0 && id < TAGS.length;
    }

    public static Pose3d getTag(int id) {
        return TAGS[id].getPose();
    }

    public static double[] getTagLayout(Fiducial[] fiducials) {
        double[] layout = new double[fiducials.length * 7];

        for (int i = 0; i < fiducials.length; i++) {
            Fiducial tag = fiducials[i];
            layout[i * 7 + 0] = tag.getID();
            layout[i * 7 + 1] = tag.getPose().getTranslation().getX();
            layout[i * 7 + 2] = tag.getPose().getTranslation().getY();
            layout[i * 7 + 3] = tag.getPose().getTranslation().getZ();
            layout[i * 7 + 4] = Units.radiansToDegrees(tag.getPose().getRotation().getX());
            layout[i * 7 + 5] = Units.radiansToDegrees(tag.getPose().getRotation().getY());
            layout[i * 7 + 6] = Units.radiansToDegrees(tag.getPose().getRotation().getZ());
        }

        return layout;
    }
}
