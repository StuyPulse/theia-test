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
        new Fiducial(0,new Pose3d(new Translation3d(0, 0, Units.inchesToMeters(30)), new Rotation3d(Units.degreesToRadians(0),Units.degreesToRadians(0),Units.degreesToRadians(0)))),
        new Fiducial(1,new Pose3d(new Translation3d(0, Units.inchesToMeters(28.125), Units.inchesToMeters(30)), new Rotation3d(Units.degreesToRadians(0),Units.degreesToRadians(0),Units.degreesToRadians(0)))),
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
        new Translation2d(0.5, 0.001368361309),
        new Translation2d(1, 0.001890508681),
        new Translation2d(1.5, 0.003221746028),
        new Translation2d(2, 0.009352868105),
        new Translation2d(2.5, 0.009364899366),
        new Translation2d(3, 0.01467209516),
        new Translation2d(3.5, 0.01837679393),
        new Translation2d(4, 0.03000858409),
        new Translation2d(4.5, 0.03192817984)
    };

    // Theta Standard Deviation vs Distance
    Translation2d[] thetaStdDevs = new Translation2d[] {
        new Translation2d(0.5, 0.2641393115),
        new Translation2d(1, 0.4433426481),
        new Translation2d(1.5, 0.660331025),
        new Translation2d(2, 0.6924061873),
        new Translation2d(2.5, 4.624662415),
        new Translation2d(3, 8.000007273),
        new Translation2d(3.5, 6.39384055),
        new Translation2d(4, 9.670544639),
        new Translation2d(4.5, 7.576406229)
    };
}
