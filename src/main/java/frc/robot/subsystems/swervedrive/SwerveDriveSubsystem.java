package frc.robot.subsystems.swervedrive;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.function.Consumer;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.statemachine.StateMachineSubsystem;
import frc.robot.Constants.AutoBuilderConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.util.LocalADStarAK;
import frc.robot.util.PhoenixOdometryThread;

/**
 * 
 */
public class SwerveDriveSubsystem extends StateMachineSubsystem {
    // TunerConstants doesn't include these constants, so they are declared locally
    private final GyroModuleIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
    private final SwerveModule[] modules = new SwerveModule[4]; // FL, FR, BL, BR
    private final Alert gyroDisconnectedAlert = new Alert("Disconnected gyro, using kinematics as fallback.",
            AlertType.kError);

    private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(SwerveDriveConstants.getModuleTranslations());
    private Rotation2d rawGyroRotation = new Rotation2d();
    private SwerveModulePosition[] lastModulePositions = // For delta tracking
            new SwerveModulePosition[] {
                    new SwerveModulePosition(),
                    new SwerveModulePosition(),
                    new SwerveModulePosition(),
                    new SwerveModulePosition()
            };
    private SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(kinematics, rawGyroRotation,
            lastModulePositions, new Pose2d());
    private final Consumer<Pose2d> resetSimulationPoseCallBack;

    private double speedModifier = 1.0;
    private boolean slowMode = false;

    // Maximum angular velocity robot can reach (rad/s)
    double maxAngularVelocity = getMaxAngularSpeedRadPerSec();

    // Feedforward gain for rotation
    double kVTheta = 12.0 / maxAngularVelocity;

    // used to track robot rotation in simulated gyro
    private double simulatedYaw;

    /**
     * 
     */
    public SwerveDriveSubsystem(GyroModuleIO gyroIO, SwerveModuleIO flModuleIO, SwerveModuleIO frModuleIO,
            SwerveModuleIO blModuleIO, SwerveModuleIO brModuleIO,
            Consumer<Pose2d> resetSimulationPoseCallBack) {
        super("SwerveDrive");

        this.gyroIO = gyroIO;
        this.resetSimulationPoseCallBack = resetSimulationPoseCallBack;
        this.modules[0] = new SwerveModule(flModuleIO, 0, TunerConstants.FrontLeft);
        this.modules[1] = new SwerveModule(frModuleIO, 1, TunerConstants.FrontRight);
        this.modules[2] = new SwerveModule(blModuleIO, 2, TunerConstants.BackLeft);
        this.modules[3] = new SwerveModule(brModuleIO, 3, TunerConstants.BackRight);

        // Usage reporting for swerve template
        HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);

        // Start odometry thread
        PhoenixOdometryThread.getInstance().start();

        // Create drive controller for PathPlanner
        // Using PID + feedforward for smooth path following
        // Position controller: handles XY position tracking
        // Lower P and add D damping to reduce oscillations during direction changes

        double[] translationPIDs = AutoBuilderConstants.getTranslationPIDs();
        double[] rotationPIDs = AutoBuilderConstants.getRotationPIDs();
        PPHolonomicDriveController driveController = new PPHolonomicDriveController(
                new PIDConstants(translationPIDs[0], translationPIDs[1], translationPIDs[2]),
                new PIDConstants(rotationPIDs[0], rotationPIDs[1], rotationPIDs[2]));

        // Configure AutoBuilder for PathPlanner
        AutoBuilder.configure(
                this::getPose,
                this::setPose,
                this::getChassisSpeeds,
                this::runVelocity,
                driveController,
                RobotConstants.PP_CONFIG,
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                this);

        Pathfinding.setPathfinder(new LocalADStarAK());
        PathPlannerLogging.setLogActivePathCallback(
                (activePath) -> {
                    Logger.recordOutput(
                            "PathPlanner/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
                });
        PathPlannerLogging.setLogTargetPoseCallback(
                (targetPose) -> {
                    Logger.recordOutput("PathPlanner/TargetPose", targetPose);
                    // Log target heading for debugging
                    double targetHeadingDegrees = targetPose.getRotation().getDegrees();
                    double targetHeadingRadians = targetPose.getRotation().getRadians();
                    Logger.recordOutput("PathPlanner/TargetHeading", targetHeadingDegrees);
                    Logger.recordOutput("PathPlanner/TargetHeadingRadians", targetHeadingRadians);
                });

        // Set initial simulated yaw
        this.simulatedYaw = 0.0;

        // set shuffleboard entries if in tuning mode
        if (RobotConstants.TUNING_MODE) {
            setShuffleboard();
            setShuffleboardTab();
        }
    }

    @Override
    public void periodic() {
        SwerveDriveConstants.odometryLock.lock(); // Prevents odometry updates while reading data
        gyroIO.updateInputs(gyroInputs);
        Logger.processInputs("SwerveDrive/Gyro", gyroInputs);
        for (var module : modules) {
            module.periodic();
        }
        SwerveDriveConstants.odometryLock.unlock();

        // Stop moving when disabled
        if (DriverStation.isDisabled()) {
            for (var module : modules) {
                module.stop();
            }
        }

        // Log empty setpoint states when disabled
        if (DriverStation.isDisabled()) {
            Logger.recordOutput("Subsystems/SwerveDrive/SwerveStates/Setpoints", new SwerveModuleState[] {});
            Logger.recordOutput("Subsystems/SwerveDrive/SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
        }

        // Update odometry
        double[] sampleTimestamps = modules[0].getOdometryTimestamps(); // All signals are sampled together
        int sampleCount = sampleTimestamps.length;
        for (int i = 0; i < sampleCount; i++) {
            // Read wheel positions and deltas from each module
            SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
            SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
            for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
                modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
                moduleDeltas[moduleIndex] = new SwerveModulePosition(
                        modulePositions[moduleIndex].distanceMeters
                                - lastModulePositions[moduleIndex].distanceMeters,
                        modulePositions[moduleIndex].angle);
                lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
            }

            // Update gyro angle
            if (gyroInputs.data.connected()) {
                // Use the real gyro angle
                rawGyroRotation = gyroInputs.data.odometryYawPositions()[i];
            } else {
                // Use the angle delta from the kinematics and module deltas
                Twist2d twist = kinematics.toTwist2d(moduleDeltas);
                rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
            }

            // Apply update
            poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
        }

        // Update gyro alert
        gyroDisconnectedAlert
                .set(!gyroInputs.data.connected() && RobotConstants.currentMode != RobotConstants.Mode.SIM);
    }

    /** Adds a new timestamped vision measurement. */
    public void addVisionMeasurement(
            Pose2d visionRobotPoseMeters,
            double timestampSeconds,
            Matrix<N3, N1> visionMeasurementStdDevs) {
        poseEstimator.addVisionMeasurement(
                visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
    }

    /** Returns a command that aligns all wheels forward and stops */
    public Command alignWheelsForward() {
        return run(() -> {
            // Create setpoint where all wheels point forward (0 degrees)
            SwerveModuleState[] forwardStates = new SwerveModuleState[] {
                    new SwerveModuleState(0, new Rotation2d(0)),
                    new SwerveModuleState(0, new Rotation2d(0)),
                    new SwerveModuleState(0, new Rotation2d(0)),
                    new SwerveModuleState(0, new Rotation2d(0))
            };

            // Send to modules
            for (int i = 0; i < 4; i++) {
                modules[i].runSetpoint(forwardStates[i]);
            }
        }).withTimeout(1.0).finallyDo(() -> stop());
    }

    /** Returns the measured chassis speeds of the robot. */
    @AutoLogOutput(key = "Subsystems/SwerveDrive/SwerveChassisSpeeds/Measured")
    public ChassisSpeeds getChassisSpeeds() {
        return kinematics.toChassisSpeeds(getModuleStates());
    }

    /**
     * 
     */
    public double getCurrentDrawAmps() {
        double getCurrentDrawAmps = 0.0;

        for (int i = 0; i < 4; i++) {
            getCurrentDrawAmps += modules[i].getCurrentDrawAmps();
        }

        return getCurrentDrawAmps;
    }

    /** Returns the maximum linear speed in meters per sec. */
    public double getMaxLinearSpeedMetersPerSec() {
        return TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    }

    /** Returns the maximum angular speed in radians per sec. */
    public double getMaxAngularSpeedRadPerSec() {
        return getMaxLinearSpeedMetersPerSec() / SwerveDriveConstants.DRIVE_BASE_RADIUS;
    }

    /**
     * Returns the module states (turn angles and drive velocities) for all of the
     * modules.
     */
    @AutoLogOutput(key = "Subsystems/SwerveDrive/SwerveStates/Measured")
    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            states[i] = modules[i].getState();
        }
        return states;
    }

    /**
     * Returns the module positions (turn angles and drive positions) for all of the
     * modules.
     */
    private SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] states = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            states[i] = modules[i].getPosition();
        }
        return states;
    }

    /**
     * 
     */
    public Rotation2d[] getModuleAngles() {
        Rotation2d[] angles = new Rotation2d[4];
        for (int i = 0; i < 4; i++) {
            angles[i] = modules[i].getAngle();
        }
        return angles;
    }

    /** Returns the current odometry pose. */
    @AutoLogOutput(key = "Subsystems/SwerveDrive/Odometry/Robot")
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /** Returns the current odometry rotation. */
    public Rotation2d getRotation() {
        return getPose().getRotation();
    }

    /**
     * 
     */
    public double getSpeedModifier() {
        return speedModifier * (slowMode ? 0.1 : 1.0);
    }

    /** Returns the position of each module in radians. */
    public double[] getWheelRadiusCharacterizationPositions() {
        double[] values = new double[4];
        for (int i = 0; i < 4; i++) {
            values[i] = modules[i].getWheelRadiusCharacterizationPosition();
        }
        return values;
    }

    public boolean isSlowMode() {
        return slowMode;
    }

    public boolean isTilted() {
        return Math.abs(gyroInputs.data.pitchDegrees()) > 5.0 || Math.abs(gyroInputs.data.rollDegrees()) > 5.0;
    }

    /**
     * Runs the drive at the desired velocity.
     *
     * @param speeds Speeds in meters/sec
     */
    public void runVelocity(ChassisSpeeds speeds) {
        // Calculate module setpoints
        ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
        SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, TunerConstants.kSpeedAt12Volts);

        // Log unoptimized setpoints and setpoint speeds
        Logger.recordOutput("Subsystems/SwerveDrive/SwerveStates/Setpoints", setpointStates);
        Logger.recordOutput("Subsystems/SwerveDrive/SwerveChassisSpeeds/Setpoints", discreteSpeeds);

        // Send setpoints to modules
        for (int i = 0; i < 4; i++) {
            modules[i].runSetpoint(setpointStates[i]);
        }

        // Log optimized setpoints (runSetpoint mutates each state)
        Logger.recordOutput("Subsystems/SwerveDrive/SwerveStates/SetpointsOptimized", setpointStates);
    }

    public void setSlowMode(boolean slowMode) {
        this.slowMode = slowMode;
    }

    /** Stops the drive. */
    public void stop() {
        runVelocity(new ChassisSpeeds());
    }

    /**
     * Stops the drive and turns the modules to an X arrangement to resist movement.
     * The modules will
     * return to their normal orientations the next time a nonzero velocity is
     * requested.
     */
    public void stopWithX() {
        Rotation2d[] headings = new Rotation2d[4];
        for (int i = 0; i < 4; i++) {
            headings[i] = SwerveDriveConstants.getModuleTranslations()[i].getAngle();
        }
        kinematics.resetHeadings(headings);
        stop();
    }

    /** Resets the current odometry pose. */
    public void setPose(Pose2d pose) {
        resetSimulationPoseCallBack.accept(pose);
        poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
    }

    @Override
    public void simulationPeriodic() {
        ChassisSpeeds robotRelativeSpeeds = getChassisSpeeds();

        double angleIncrease = robotRelativeSpeeds.omegaRadiansPerSecond * 0.2;
        simulatedYaw += angleIncrease;

        if (gyroIO instanceof GyroModuleIOSim gyroModuleIOSim) {
            gyroModuleIOSim.setRawYaw(angleIncrease);
            // double rawYawRadians = getRotation().getRadians();
            // gyroModuleIOSim.setRawYaw(rawYawRadians);
        }
    }

    /**
     * 
     */
    @Override
    protected void setShuffleboard() {
        if (modules == null)
            return;

        for (int i = 0; i < 4; i++) {
            modules[i].setShuffleboard();
        }
    }

    @Override
    protected void setShuffleboardTab() {
        if (modules == null)
            return;

        for (int i = 0; i < 4; i++) {
            modules[i].setShuffleboardTab();
        }
    }

    @Override
    protected void setShuffleboardEntries() {
        if (modules == null)
            return;

        for (int i = 0; i < 4; i++) {
            modules[i].setShuffleboardEntries();
        }
    }

    /**
     * Overrides for SysId routines
     */
    @Override
    public double getFFCharacterizationVelocity() {
        double output = 0.0;
        for (int i = 0; i < 4; i++) {
            output += modules[i].getFFCharacterizationVelocity() / 4.0;
        }
        return output;
    }

    @Override
    public void runCharacterization(double output) {
        for (int i = 0; i < 4; i++) {
            modules[i].runCharacterization(output);
        }
    }
}
