package com.stuypulse.robot.subsystems.swerve.modules;

import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.angle.AngleController;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.control.feedforward.MotorFeedforward;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.streams.angles.filters.ARateLimit;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Swerve;
import com.stuypulse.robot.constants.Settings.Swerve.Drive;
import com.stuypulse.robot.constants.Settings.Swerve.Turn;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SimModule extends AbstractSwerveModule {

    private static LinearSystem<N2, N1, N2> identifyVelocityPositionSystem(double kV, double kA) {
        if (kV <= 0.0) {
            throw new IllegalArgumentException("Kv must be greater than zero.");
          }
          if (kA <= 0.0) {
            throw new IllegalArgumentException("Ka must be greater than zero.");
          }

          return new LinearSystem<N2, N1, N2>(
              Matrix.mat(Nat.N2(), Nat.N2()).fill(0.0, 1.0, 0.0, -kV / kA),
              Matrix.mat(Nat.N2(), Nat.N1()).fill(0.0, 1.0 / kA),
              Matrix.mat(Nat.N2(), Nat.N2()).fill(1.0, 0.0, 0.0, 1.0),
              Matrix.mat(Nat.N2(), Nat.N1()).fill(0.0, 0.0));
    }

    private final String ID;
    private final Translation2d moduleLocation;
    private SwerveModuleState targetState;

    private final LinearSystemSim<N2, N1, N1> turnSim;

    private final LinearSystemSim<N2, N1, N2> driveSim;

    private Controller driveController;
    private AngleController turnController;

    public SimModule(String ID, Translation2d moduleLocation) {

        this.ID = ID;
        this.moduleLocation = moduleLocation;

        turnSim = new LinearSystemSim<>(LinearSystemId.identifyPositionSystem(Turn.kV.doubleValue(), Turn.kA.doubleValue()));
        turnController = new AnglePIDController(Turn.kP, Turn.kI, Turn.kD)
            .setSetpointFilter(new ARateLimit(Swerve.MAX_TURNING));

        driveSim = new LinearSystemSim<>(identifyVelocityPositionSystem(Drive.kV.doubleValue(), Drive.kA.doubleValue()));
        driveController = new PIDController(Drive.kP, Drive.kI, Drive.kD)
            .add(new MotorFeedforward(Drive.kS, Drive.kV, Drive.kA).velocity());

        targetState = new SwerveModuleState();
    }

    @Override
    public String getID() {
        return ID;
    }

    @Override
    public Translation2d getModuleLocation() {
        return moduleLocation;
    }

    @Override
    public SwerveModuleState getState() {
        return new SwerveModuleState(getVelocity(), getAngle());
    }

    private double getVelocity() {
        return driveSim.getOutput(1);
    }

    private double getDistance() {
        return driveSim.getOutput(0);
    }

    private Rotation2d getAngle() {
        return Rotation2d.fromRadians(turnSim.getOutput(0));
    }

    @Override
    public Rotation2d getWheelRotationOffset() {
        return new Rotation2d();
    }

    @Override
    public void setTargetState(SwerveModuleState state) {
        targetState = SwerveModuleState.optimize(state, getAngle());
    }

    @Override
    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(getDistance(), getAngle());
    }

    @Override
    public void periodic() {

        turnController.update(Angle.fromRotation2d(targetState.angle), Angle.fromRotation2d(getAngle()));
        driveController.update(targetState.speedMetersPerSecond, getVelocity());

        SmartDashboard.putNumber("Swerve/" + ID + "/Target Angle", targetState.angle.getDegrees());
        SmartDashboard.putNumber("Swerve/" + ID + "/Angle", getAngle().getDegrees());
        SmartDashboard.putNumber("Swerve/" + ID + "/Angle Error", turnController.getError().toDegrees());
        SmartDashboard.putNumber("Swerve/" + ID + "/Angle Voltage", turnController.getOutput());
        SmartDashboard.putNumber("Swerve/" + ID + "/Angle Current", turnSim.getCurrentDrawAmps());
        
        SmartDashboard.putNumber("Swerve/" + ID + "/Target Velocity", targetState.speedMetersPerSecond);
        SmartDashboard.putNumber("Swerve/" + ID + "/Velocity", getVelocity());
        SmartDashboard.putNumber("Swerve/" + ID + "/Velocity Error", driveController.getError());
        SmartDashboard.putNumber("Swerve/" + ID + "/Velocity Voltage", driveController.getOutput());
        SmartDashboard.putNumber("Swerve/" + ID + "/Velocity Current", driveSim.getCurrentDrawAmps());
    }

    @Override
    public void simulationPeriodic() {
        driveSim.setInput(driveController.getOutput());
        driveSim.update(Settings.DT);

        turnSim.setInput(turnController.getOutput());
        turnSim.update(Settings.DT);

       RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(
           turnSim.getCurrentDrawAmps() + driveSim.getCurrentDrawAmps()
       ));
    }
}