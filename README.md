# Theia Test

This robot project is to be used for testing the [Theia](https://github.com/anivanchen/aruco) custom camera localization solution. 

### Swerve Drive

The SwerveDrive class handles the interactions with the swerve drive hardware. It is responsible for calculating the wheel angles and speeds, and setting the motors to the correct values.

### Odometry

The Odometry class handles the localization of the robot. It uses the wheel angles and speeds to calculate the robot's position and orientation. It also uses a pose estimator to innclude data from the Vision subsystem to correct for drift.

### Vision

The Vision class manages the multiple cameras on the robot along with the data from these cameras.

### CustomCamera

The CustomCamera class handles the interactions with the custom camera hardware via NetworkTables4. It is responsible for posting and receiving data to and from the camera.

### LinearRegression

The LinearRegression class handles the linear regression calculations to determine stddev values for vision pose extimates in the pose estimator.

### Fiducial

The Fiducial class stores the data for a single fiducial.

- ID
- Pose

### VisionData

The VisionData class stores the data representing a frame of vision data from a camera.

- IDs
- Tag Poses relative to camera
- Camera Pose relative to robot
- Robot Pose relative to field
- Latency

