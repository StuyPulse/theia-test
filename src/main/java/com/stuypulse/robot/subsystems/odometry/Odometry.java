package com.stuypulse.robot.subsystems.odometry;

import java.util.List;

import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.robot.subsystems.vision.AbstractVision;
import com.stuypulse.robot.util.VisionData;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Odometry extends AbstractOdometry {

    private final SwerveDrivePoseEstimator estimator;
    private final SwerveDriveOdometry odometry;

    private final Field2d field;
    private final FieldObject2d odometryPose2D;
    private final FieldObject2d estimatorPose2D;

    Vector<N3> visionStdDevs = VecBuilder.fill(1, 1, Units.degreesToRadians(30));

    protected Odometry() {
        SwerveDrive swerve = SwerveDrive.getInstance();
        
        this.odometry = new SwerveDriveOdometry(
            swerve.getKinematics(), 
            swerve.getGyroYaw(), 
            swerve.getModulePositions(), 
            new Pose2d());

        this.estimator = new SwerveDrivePoseEstimator(
            swerve.getKinematics(), 
            swerve.getGyroYaw(), 
            swerve.getModulePositions(), 
            new Pose2d(),
            VecBuilder.fill(0.1, 0.1, 0.1),
            visionStdDevs);

        this.field = new Field2d();
        this.odometryPose2D = field.getObject("Odometry Pose");
        this.estimatorPose2D = field.getObject("Estimator Pose");

        swerve.initFieldObjects(field);
        SmartDashboard.putData("Field", field);
    }

    @Override
    public Field2d getField() {
        return field;
    }

    @Override
    public Pose2d getPose() {
        return new Pose2d(estimator.getEstimatedPosition().getTranslation(), odometry.getPoseMeters().getRotation());
    }

    @Override
    public void reset(Pose2d pose2d) {
        SwerveDrive swerve = SwerveDrive.getInstance();
        
        odometry.resetPosition(swerve.getGyroYaw(), swerve.getModulePositions(), pose2d);
        estimator.resetPosition(swerve.getGyroYaw(), swerve.getModulePositions(), pose2d);
    }

    private void updateWithVision(List<VisionData> visionData) {
        for (VisionData result : visionData) {
            estimator.addVisionMeasurement(
                result.robotPose.toPose2d(), 
                Timer.getFPGATimestamp() - result.latency,
                visionStdDevs
            );
        }
    }

    @Override
    public void periodic() {
        SwerveDrive swerve = SwerveDrive.getInstance();
        
        odometry.update(swerve.getGyroYaw(), swerve.getModulePositions());
        estimator.update(swerve.getGyroYaw(), swerve.getModulePositions());

        List<VisionData> output = AbstractVision.getInstance().getOutput();
        updateWithVision(output);

        odometryPose2D.setPose(odometry.getPoseMeters());
        estimatorPose2D.setPose(estimator.getEstimatedPosition());

        SmartDashboard.putNumber("Odometry/Odometry/X", odometry.getPoseMeters().getTranslation().getX());
        SmartDashboard.putNumber("Odometry/Odometry/Y", odometry.getPoseMeters().getTranslation().getY());
        SmartDashboard.putNumber("Odometry/Odometry/Rotation", odometry.getPoseMeters().getRotation().getDegrees());

        SmartDashboard.putNumber("Odometry/Estimator/X", estimator.getEstimatedPosition().getTranslation().getX());
        SmartDashboard.putNumber("Odometry/Estimator/Y", estimator.getEstimatedPosition().getTranslation().getY());
        SmartDashboard.putNumber("Odometry/Estimator/Rotation", estimator.getEstimatedPosition().getRotation().getDegrees());
    }
}