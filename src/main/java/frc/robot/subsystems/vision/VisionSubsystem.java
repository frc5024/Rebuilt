package frc.robot.subsystems.vision;

import java.util.LinkedList;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.swervedrive.SwerveDriveSubsystem;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;

/**
 * 
 */
public class VisionSubsystem extends SubsystemBase {
    private final VisionConsumer consumer;
    private final VisionIO[] visionIO;
    private final VisionIOInputsAutoLogged[] inputs;
    private final Alert[] disconnectedAlerts;
    private final SwerveDriveSubsystem drivetrain;

    /**
     * 
     */
    public VisionSubsystem(VisionConsumer consumer, SwerveDriveSubsystem drivetrain, VisionIO... visionIO) {
        this.consumer = consumer;
        this.drivetrain = drivetrain;
        this.visionIO = visionIO;

        // Initialize inputs
        this.inputs = new VisionIOInputsAutoLogged[visionIO.length];
        for (int i = 0; i < this.inputs.length; i++) {
            this.inputs[i] = new VisionIOInputsAutoLogged();
        }

        // Initialize disconnected alerts
        this.disconnectedAlerts = new Alert[visionIO.length];
        for (int i = 0; i < this.inputs.length; i++) {
            this.disconnectedAlerts[i] = new Alert("Vision camera " + this.visionIO[i].getName() + " is disconnected.",
                    AlertType.kWarning);
        }
    }

    /**
     * Returns the X angle to the best target, which can be used for simple servoing
     * with vision.
     *
     * @param cameraIndex The index of the camera to use.
     */
    public Rotation2d getTargetX(int cameraIndex) {
        return this.inputs[cameraIndex].latestTargetObservation.tx();
    }

    @Override
    public void periodic() {
        // Disable vision during autonomous to prevent interference with PathPlanner
        // if (DriverStation.isAutonomous()) {
        // return;
        // }

        for (int i = 0; i < this.visionIO.length; i++) {
            this.visionIO[i].updateInputs(this.inputs[i]);
            Logger.processInputs("Vision/" + this.visionIO[i].getName(), this.inputs[i]);
        }

        // Initialize logging values
        List<Pose3d> allTagPoses = new LinkedList<>();
        List<Pose3d> allRobotPoses = new LinkedList<>();
        List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
        List<Pose3d> allRobotPosesRejected = new LinkedList<>();

        // Loop over cameras
        for (int cameraIndex = 0; cameraIndex < this.visionIO.length; cameraIndex++) {
            // Update disconnected alert
            this.disconnectedAlerts[cameraIndex].set(!this.inputs[cameraIndex].connected);

            // Initialize logging values
            List<Pose3d> tagPoses = new LinkedList<>();
            List<Pose3d> robotPoses = new LinkedList<>();
            List<Pose3d> robotPosesAccepted = new LinkedList<>();
            List<Pose3d> robotPosesRejected = new LinkedList<>();

            // Add tag poses
            for (int tagId : this.inputs[cameraIndex].tagIds) {
                var tagPose = VisionConstants.aprilTagLayout.getTagPose(tagId);
                if (tagPose.isPresent()) {
                    tagPoses.add(tagPose.get());
                }
            }

            // Loop over pose observations
            for (var observation : this.inputs[cameraIndex].poseObservations) {
                // Check whether to reject pose
                boolean rejectPose = observation.tagCount() == 0 // Must have at least one tag
                        || (observation.tagCount() == 1
                                && observation.ambiguity() > VisionConstants.maxAmbiguity) // Cannot be high ambiguity
                        || Math.abs(observation.pose().getZ()) > VisionConstants.maxZError // Must have realistic Z
                                                                                           // coordinate

                        // Must be within the field boundaries
                        || observation.pose().getX() < 0.0
                        || observation.pose().getX() > VisionConstants.aprilTagLayout.getFieldLength()
                        || observation.pose().getY() < 0.0
                        || observation.pose().getY() > VisionConstants.aprilTagLayout.getFieldWidth();

                // Reject if robot rotating fast
                var chassisSpeeds = drivetrain.getChassisSpeeds();
                if (Math.abs(chassisSpeeds.omegaRadiansPerSecond) > Math.toRadians(120)) {
                    continue;
                }

                // Reject if robot moving fast
                double linearSpeed = Math.hypot(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);
                if (linearSpeed > 3.0) {
                    continue;
                }

                // Add pose to log
                robotPoses.add(observation.pose());
                if (rejectPose) {
                    robotPosesRejected.add(observation.pose());
                } else {
                    robotPosesAccepted.add(observation.pose());
                }

                // Skip if rejected
                if (rejectPose) {
                    continue;
                }

                // Calculate standard deviations
                double stdDevFactor = Math.pow(observation.averageTagDistance(), 4.0) / observation.tagCount();
                double linearStdDev = VisionConstants.linearStdDevBaseline * stdDevFactor;
                double angularStdDev = VisionConstants.angularStdDevBaseline * stdDevFactor;
                if (observation.type() == PoseObservationType.MEGATAG_2) {
                    linearStdDev *= VisionConstants.linearStdDevMegatag2Factor;
                    angularStdDev *= VisionConstants.angularStdDevMegatag2Factor;
                }
                if (cameraIndex < VisionConstants.cameraStdDevFactors.length) {
                    linearStdDev *= VisionConstants.cameraStdDevFactors[cameraIndex];
                    angularStdDev *= VisionConstants.cameraStdDevFactors[cameraIndex];
                }

                // During autonomous, heavily weight toward position and away from rotation
                // This lets vision correct X,Y position but not interfere with PathPlanner's
                // heading
                if (DriverStation.isAutonomous()) {
                    angularStdDev = 10000.0; // Extremely high = ignore rotation, only use position during auto
                }

                // Send vision observation
                this.consumer.accept(
                        observation.pose().toPose2d(),
                        observation.timestamp(),
                        VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
            }

            // Log camera datadata
            Logger.recordOutput(
                    "Vision/" + this.visionIO[cameraIndex].getName() + "/TagPoses",
                    tagPoses.toArray(new Pose3d[tagPoses.size()]));
            Logger.recordOutput(
                    "Vision/" + this.visionIO[cameraIndex].getName() + "/RobotPoses",
                    robotPoses.toArray(new Pose3d[robotPoses.size()]));
            Logger.recordOutput(
                    "Vision/" + this.visionIO[cameraIndex].getName() + "/RobotPosesAccepted",
                    robotPosesAccepted.toArray(new Pose3d[robotPosesAccepted.size()]));
            Logger.recordOutput(
                    "Vision/" + this.visionIO[cameraIndex].getName() + "/RobotPosesRejected",
                    robotPosesRejected.toArray(new Pose3d[robotPosesRejected.size()]));

            allTagPoses.addAll(tagPoses);
            allRobotPoses.addAll(robotPoses);
            allRobotPosesAccepted.addAll(robotPosesAccepted);
            allRobotPosesRejected.addAll(robotPosesRejected);
        }

        // Log summary data
        Logger.recordOutput("Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[allTagPoses.size()]));
        Logger.recordOutput("Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[allRobotPoses.size()]));
        Logger.recordOutput("Vision/Summary/RobotPosesAccepted",
                allRobotPosesAccepted.toArray(new Pose3d[allRobotPosesAccepted.size()]));
        Logger.recordOutput("Vision/Summary/RobotPosesRejected",
                allRobotPosesRejected.toArray(new Pose3d[allRobotPosesRejected.size()]));
    }

    @FunctionalInterface
    public static interface VisionConsumer {
        public void accept(
                Pose2d visionRobotPoseMeters,
                double timestampSeconds,
                Matrix<N3, N1> visionMeasurementStdDevs);
    }
}
