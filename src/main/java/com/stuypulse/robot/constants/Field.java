/************************ PROJECT OFSS ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.constants;

import com.stuypulse.robot.util.Fiducial;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public interface Field {

    public static final double FIDUCIAL_SIZE = 0.15558;

    Fiducial TAGS[] = {
        new Fiducial(0,new Pose3d(new Translation3d(0, 4, 0), new Rotation3d(Units.degreesToRadians(0),Units.degreesToRadians(0),Units.degreesToRadians(180)))),
        
        new Fiducial(1,new Pose3d(new Translation3d(15.513, 1.072, 0), new Rotation3d(Units.degreesToRadians(0),Units.degreesToRadians(0),Units.degreesToRadians(180)))),
        new Fiducial(2,new Pose3d(new Translation3d(15.513, 2.75, 0), new Rotation3d(Units.degreesToRadians(0),Units.degreesToRadians(0),Units.degreesToRadians(180)))),
        new Fiducial(3,new Pose3d(new Translation3d(15.513, 4.42, 0), new Rotation3d(Units.degreesToRadians(0),Units.degreesToRadians(0),Units.degreesToRadians(180)))),
        new Fiducial(4,new Pose3d(new Translation3d(16.18, 6.75, 0), new Rotation3d(Units.degreesToRadians(0),Units.degreesToRadians(0),Units.degreesToRadians(180)))),

        new Fiducial(5,new Pose3d(new Translation3d(0.36, 6.75, 0), new Rotation3d(Units.degreesToRadians(0),Units.degreesToRadians(0), Units.degreesToRadians(0)))),
        new Fiducial(6,new Pose3d(new Translation3d(1.027, 4.42, 0), new Rotation3d(Units.degreesToRadians(0),Units.degreesToRadians(0), Units.degreesToRadians(0)))),
        new Fiducial(7,new Pose3d(new Translation3d(1.027, 2.75, 0), new Rotation3d(Units.degreesToRadians(0),Units.degreesToRadians(0), Units.degreesToRadians(0)))),
        new Fiducial(8,new Pose3d(new Translation3d(1.027, 1.072, 0), new Rotation3d(Units.degreesToRadians(0),Units.degreesToRadians(0), Units.degreesToRadians(0)))),
    };

    public static boolean isValidTag(int id) {
        for (Fiducial tag : TAGS)
            if (tag.getID() == id) return true;
        return false;
    }

    public static Fiducial getTag(int id) {
        for (Fiducial tag : TAGS)
            if (tag.getID() == id) return tag;
        return null;
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

    // XY Standard Deviation vs Distance
    Translation2d[] xyStdDevs = new Translation2d[] {
        new Translation2d(1, 1),
        new Translation2d(2, 2),
        new Translation2d(3, 3),
        new Translation2d(4, 4),
        new Translation2d(5, 5),
        new Translation2d(6, 6),
        new Translation2d(7, 7),
        new Translation2d(8, 8),
    };

    // Theta Standard Deviation vs Distance
    Translation2d[] thetaStdDevs = new Translation2d[] {
        new Translation2d(1, 1),
        new Translation2d(2, 2),
        new Translation2d(3, 3),
        new Translation2d(4, 4),
        new Translation2d(5, 5),
        new Translation2d(6, 6),
        new Translation2d(7, 7),
        new Translation2d(8, 8),
    };
}
