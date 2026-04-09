package frc.robot.subsystems.vision;

import java.util.LinkedList;
import java.util.List;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
import frc.robot.util.LoggedTracer;

/**
 * 
 */
public class VisionSubsystem extends SubsystemBase {
    // Advantagekit logging
    private final VisionIO[] visionIO;
    private final VisionIOInputsAutoLogged[] inputs;

    // Parameters
    private final VisionConsumer consumer;
    private final Supplier<ChassisSpeeds> chassisSpeedsSupplier;

    // Variables
    private final StringBuilder sb;

    // Lists used to store the poses
    private final List<Pose3d> allTagPoses;
    private final List<Pose3d> allRobotPoses;
    private final List<Pose3d> allRobotPosesAccepted;
    private final List<Pose3d> allRobotPosesRejected;

    private final List<Pose3d> tagPoses;
    private final List<Pose3d> robotPoses;
    private final List<Pose3d> robotPosesAccepted;
    private final List<Pose3d> robotPosesRejected;

    // Alerts
    private final Alert[] disconnectedAlerts;

    /**
     * 
     */
    public VisionSubsystem(VisionConsumer consumer, Supplier<ChassisSpeeds> chassisSpeedsSupplier,
            VisionIO... visionIO) {
        this.consumer = consumer;
        this.chassisSpeedsSupplier = chassisSpeedsSupplier;
        this.visionIO = visionIO;

        // Initialize inputs
        this.inputs = new VisionIOInputsAutoLogged[visionIO.length];
        for (int i = 0; i < this.inputs.length; i++) {
            this.inputs[i] = new VisionIOInputsAutoLogged();
        }

        // Initialize pose lists
        this.allTagPoses = new LinkedList<>();
        this.allRobotPoses = new LinkedList<>();
        this.allRobotPosesAccepted = new LinkedList<>();
        this.allRobotPosesRejected = new LinkedList<>();

        this.tagPoses = new LinkedList<>();
        this.robotPoses = new LinkedList<>();
        this.robotPosesAccepted = new LinkedList<>();
        this.robotPosesRejected = new LinkedList<>();

        // Initialize disconnected alerts
        this.disconnectedAlerts = new Alert[visionIO.length];
        for (int i = 0; i < this.inputs.length; i++) {
            this.disconnectedAlerts[i] = new Alert("Vision camera " + this.visionIO[i].getName() + " is disconnected.",
                    AlertType.kWarning);
        }

        this.sb = new StringBuilder();
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
        for (int i = 0; i < this.visionIO.length; i++) {
            this.visionIO[i].updateInputs(this.inputs[i]);
            Logger.processInputs("Vision/" + this.visionIO[i].getName(), this.inputs[i]);
        }

        // Clear logging values
        allTagPoses.clear();
        allRobotPoses.clear();
        allRobotPosesAccepted.clear();
        allRobotPosesRejected.clear();

        // Loop over cameras
        for (int cameraIndex = 0; cameraIndex < this.visionIO.length; cameraIndex++) {
            // Update disconnected alert
            this.disconnectedAlerts[cameraIndex].set(!this.inputs[cameraIndex].connected);

            // Clear logging values
            tagPoses.clear();
            robotPoses.clear();
            robotPosesAccepted.clear();
            robotPosesRejected.clear();

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
                // Must have at least one tag, cannot be high ambiguity, must have realistic Z
                // coordinate, must be within the field boundaries
                boolean rejectPose = observation.tagCount() == 0
                        || (observation.tagCount() == 1 && observation.ambiguity() > VisionConstants.maxAmbiguity)
                        || Math.abs(observation.pose().getZ()) > VisionConstants.maxZError
                        || observation.pose().getX() < 0.0
                        || observation.pose().getX() > VisionConstants.aprilTagLayout.getFieldLength()
                        || observation.pose().getY() < 0.0
                        || observation.pose().getY() > VisionConstants.aprilTagLayout.getFieldWidth();

                // Reject if robot rotating fast
                var chassisSpeeds = chassisSpeedsSupplier.get();
                if (Math.abs(chassisSpeeds.omegaRadiansPerSecond) > Math.toRadians(120)) {
                    rejectPose = true;
                }

                // Reject if robot moving fast
                double linearSpeed = Math.hypot(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);
                if (linearSpeed > 3.0) {
                    rejectPose = true;
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
            sb.setLength(0);
            sb.append("Subsystems/Vision/").append(visionIO[cameraIndex].getName()).append("/TagPoses");
            Logger.recordOutput(sb.toString(), tagPoses.toArray(new Pose3d[tagPoses.size()]));

            sb.setLength(0);
            sb.append("Subsystems/Vision/").append(visionIO[cameraIndex].getName()).append("/RobotPoses");
            Logger.recordOutput(sb.toString(), robotPoses.toArray(new Pose3d[robotPoses.size()]));

            sb.setLength(0);
            sb.append("Subsystems/Vision/").append(visionIO[cameraIndex].getName()).append("/RobotPosesAccepted");
            Logger.recordOutput(sb.toString(), robotPosesAccepted.toArray(new Pose3d[robotPosesAccepted.size()]));

            sb.setLength(0);
            sb.append("Subsystems/Vision/").append(visionIO[cameraIndex].getName()).append("/RobotPosesRejected");
            Logger.recordOutput(sb.toString(), robotPosesRejected.toArray(new Pose3d[robotPosesRejected.size()]));

            allTagPoses.addAll(tagPoses);
            allRobotPoses.addAll(robotPoses);
            allRobotPosesAccepted.addAll(robotPosesAccepted);
            allRobotPosesRejected.addAll(robotPosesRejected);
        }

        // Log summary data
        sb.setLength(0);
        sb.append("Subsystems/Vision/Summary/TagPoses");
        Logger.recordOutput(sb.toString(), allTagPoses.toArray(new Pose3d[allTagPoses.size()]));

        sb.setLength(0);
        sb.append("Subsystems/Vision/Summary/RobotPoses");
        Logger.recordOutput(sb.toString(), allRobotPoses.toArray(new Pose3d[allRobotPoses.size()]));

        sb.setLength(0);
        sb.append("Subsystems/Vision/Summary/RobotPosesAccepted");
        Logger.recordOutput(sb.toString(), allRobotPosesAccepted.toArray(new Pose3d[allRobotPosesAccepted.size()]));

        sb.setLength(0);
        sb.append("Subsystems/Vision/Summary/RobotPosesRejected");
        Logger.recordOutput(sb.toString(), allRobotPosesRejected.toArray(new Pose3d[allRobotPosesRejected.size()]));

        // Record cycle time
        LoggedTracer.record("Vision");
    }

    @FunctionalInterface
    public static interface VisionConsumer {
        public void accept(
                Pose2d visionRobotPoseMeters,
                double timestampSeconds,
                Matrix<N3, N1> visionMeasurementStdDevs);
    }
}
