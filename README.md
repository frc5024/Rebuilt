# Rebuilt
Team 5024 Robot Code
# Vision Subsystem
The vision subsystem has the primary function to provide the robot the information of where it is located on the field by using AprilTags, miniature QR-codes placed in certain places on the field that cameras can detect and measure out to determine where the robot is on the field. The robot uses that information to perform certain actions, such as driving, shooting and climbing, much more accurately.
The vision subsystem relies heavily on using Limelights to see the AprilTags. The documentation can be found here.
[Limelight Documentation](https://docs.limelightvision.io/docs/docs-limelight/getting-started/summary)
## Vision Constants
* aprilTagLayout - Determines which AprilTag positions to use, automatically updates to the most recent field each year and should not be changed unless testing for previous years
* frontCamera/rearCamera - The names and positions of the cameras on the robot, must match the names of the physical Limelights as set in the coprocessor
* maxAmbiguity, maxZError, linearStdDevBaseline, angularStdDevBaseline, cameraStdDevFactors, linearStdDevMegatag2Factor, angularStdDevMegatag2Factor - Used for finding the most accurate locations, should not be changed after being set for the first time
## Vision ToDo
* think about tuning mode
* use example code in MapleSimUtil.java and AdvantageScope Game Piece Objects help section to visualize shoot fuel into hub
* create a path that aligns the climber hook to the left side of the tower
* create a path that aligns the climber hook to the right side of the tower
* create a command called DriveNearestTunnelCommand that drives a path through the nearest tunnel to a shooting position from anywhere on the field
* update VisionIO.java to store the name of the camera so it can be used in the VisionSubsystem logging
* update the logging for SwerveDriveSubsystem and VisionSubsystem  - add Subsystem folder path to clean up AdvantageScope