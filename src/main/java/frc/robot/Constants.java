package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;

import com.ctre.phoenix6.CANBus;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.generated.TunerConstants;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always
 * "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics
 * sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
    public static final Mode simMode = Mode.SIM;
    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

    public static final double maxLinearSpeed = 4.69;
    public static final double maxLinearAcceleration = 4.0;
    public static final double maxAngularAcceleration = 20.0;
    public static final double maxAngularSpeed = 8.0; // 4.69 / driveBaseRadius;

    public static final PathConstraints CONSTRAINTS = new PathConstraints(4.5, 4.0, Units.degreesToRadians(540),
            Units.degreesToRadians(720));

    public static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(
            maxLinearSpeed,
            maxLinearAcceleration);
    public static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(
            maxLinearSpeed,
            maxLinearAcceleration);
    public static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS = new TrapezoidProfile.Constraints(
            maxAngularSpeed, maxLinearAcceleration);

    public static final Pose2d[][] STATION_POSES = new Pose2d[][] {
            {
                    // new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)),
                    new Pose2d(7.153, 7.272, Rotation2d.fromDegrees(180.0)),
                    new Pose2d(7.153, 6.169, Rotation2d.fromDegrees(180.0)),
                    new Pose2d(7.127, 1.905, Rotation2d.fromDegrees(180.0))
            },
            {
                    // new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)),
                    new Pose2d(10.447, 0.805, Rotation2d.fromDegrees(0.0)),
                    new Pose2d(10.447, 1.991, Rotation2d.fromDegrees(0.0)),
                    new Pose2d(10.447, 3.003, Rotation2d.fromDegrees(0.0))
            }
    };

    public static enum Mode {
        /** Running on a real robot. */
        REAL,

        /** Running a physics simulator. */
        SIM,

        /** Replaying from a log file. */
        REPLAY
    }

    /*
     * 
     */
    public static class RobotConstants {
        // PathPlanner config constants
        private static final double ROBOT_MASS_KG = 74.088;
        private static final double ROBOT_MOI = 6.883;
        private static final double WHEEL_COF = 1.2;
        public static final RobotConfig PP_CONFIG = new RobotConfig(
                ROBOT_MASS_KG,
                ROBOT_MOI,
                new ModuleConfig(
                        TunerConstants.FrontLeft.WheelRadius,
                        TunerConstants.kSpeedAt12Volts.in(MetersPerSecond),
                        WHEEL_COF,
                        DCMotor.getKrakenX60Foc(1)
                                .withReduction(TunerConstants.FrontLeft.DriveMotorGearRatio),
                        TunerConstants.FrontLeft.SlipCurrent,
                        1),
                SwerveDriveConstants.getModuleTranslations());
    }

    /**
     * Field Constants
     */

    /**
     * Maple Sim Constants
     */
    public static class MapleSimConstants {
        public static final int driveMotorCurrentLimit = 60;
        public static final int turnMotorCurrentLimit = 20;

        public static final double driveSimP = 0.05;
        public static final double driveSimD = 0.0;
        public static final double driveSimKs = 0.00865;
        private static final double DRIVE_KV_ROT = 0.91035; // Same units as TunerConstants: (volt * secs) /
                                                            // rotation
        public static final double driveSimKv = 1.0 / Units.rotationsToRadians(1.0 / DRIVE_KV_ROT); // 0.0789;

        public static final double turnSimP = 8.0;
        public static final double turnSimD = 0.0;

        public static final DriveTrainSimulationConfig mapleSimConfig = DriveTrainSimulationConfig.Default()
                .withRobotMass(Kilograms.of(RobotConstants.ROBOT_MASS_KG))
                .withCustomModuleTranslations(SwerveDriveConstants.getModuleTranslations())
                .withGyro(COTS.ofPigeon2())
                .withSwerveModule(
                        new SwerveModuleSimulationConfig(
                                DCMotor.getKrakenX60(1),
                                DCMotor.getFalcon500(1),
                                TunerConstants.FrontLeft.DriveMotorGearRatio,
                                TunerConstants.FrontLeft.SteerMotorGearRatio,
                                Volts.of(TunerConstants.FrontLeft.DriveFrictionVoltage),
                                Volts.of(TunerConstants.FrontLeft.SteerFrictionVoltage),
                                Inches.of(2),
                                KilogramSquareMeters.of(
                                        TunerConstants.FrontLeft.SteerInertia),
                                RobotConstants.WHEEL_COF));
    }

    /**
     * 
     */
    public static class SwerveDriveConstants {
        public static final double ODOMETRY_FREQUENCY = new CANBus(
                TunerConstants.DrivetrainConstants.CANBusName).isNetworkFD() ? 250.0 : 100.0;
        public static final double DRIVE_BASE_RADIUS = Math.max(
                Math.max(
                        Math.hypot(TunerConstants.FrontLeft.LocationX,
                                TunerConstants.FrontLeft.LocationY),
                        Math.hypot(TunerConstants.FrontRight.LocationX,
                                TunerConstants.FrontRight.LocationY)),
                Math.max(
                        Math.hypot(TunerConstants.BackLeft.LocationX,
                                TunerConstants.BackLeft.LocationY),
                        Math.hypot(TunerConstants.BackRight.LocationX,
                                TunerConstants.BackRight.LocationY)));

        public static final Lock odometryLock = new ReentrantLock();

        /** Returns an array of module translations. */
        public static Translation2d[] getModuleTranslations() {
            return new Translation2d[] {
                    new Translation2d(TunerConstants.FrontLeft.LocationX,
                            TunerConstants.FrontLeft.LocationY),
                    new Translation2d(TunerConstants.FrontRight.LocationX,
                            TunerConstants.FrontRight.LocationY),
                    new Translation2d(TunerConstants.BackLeft.LocationX,
                            TunerConstants.BackLeft.LocationY),
                    new Translation2d(TunerConstants.BackRight.LocationX,
                            TunerConstants.BackRight.LocationY)
            };
        }
    }

    public static class VisionConstants {
        // AprilTag layout
        public static AprilTagFieldLayout aprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

        // Camera names, must match names configured on coprocessor
        public static String camera0Name = "camera_0";
        public static String camera1Name = "camera_1";

        // Robot to camera transforms
        // (Not used by Limelight, configure in web UI instead)
        public static Transform3d robotToCamera0 = new Transform3d(0.2, 0.0, 0.2, new Rotation3d(0.0, -0.4, 0.0));
        public static Transform3d robotToCamera1 = new Transform3d(-0.2, 0.0, 0.2, new Rotation3d(0.0, -0.4, Math.PI));

        // Basic filtering thresholds
        public static double maxAmbiguity = 0.3;
        public static double maxZError = 0.75;

        // Standard deviation baselines, for 1 meter distance and 1 tag
        // (Adjusted automatically based on distance and # of tags)
        public static double linearStdDevBaseline = 0.02; // Meters
        public static double angularStdDevBaseline = 0.06; // Radians

        // Standard deviation multipliers for each camera
        // (Adjust to trust some cameras more than others)
        public static double[] cameraStdDevFactors = new double[] {
                1.0, // Camera 0
                1.0 // Camera 1
        };

        // Multipliers to apply for MegaTag 2 observations
        public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
        public static double angularStdDevMegatag2Factor = Double.POSITIVE_INFINITY; // No rotation data available
    }

}
