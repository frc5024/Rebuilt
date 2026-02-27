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
import frc.lib.camera.Camera;
import frc.robot.generated.TunerConstants;

/**
 * 
 */
public final class Constants {

    public static class HopperConstants {
        public static final int CAPACITY = 20;
        public static double hopperSpeed = 0.85;
        public static int HopperMotorID = 8;
    }

    public static final class turretConstants {
        public static final double speed = 0.3;
        public static final int turretMotorChannel = 3;
        public static final double turretTolerance = 0.5;
        public static final double kP = 0.2;
        public static final double kI = 0;
        public static final double kD = 0.01;
        public static final double kS = 0;
        public static final double kV = 0.0001;
        public static final double kA = 0.0;

        public static final double turretMaxSpeed = 1000; // make note of units
        public static final double turretMaxAccel = 1000;

        public static final double targetAngle = 25;

    }

    public static final double targetAngle = 25;

    public static class shooterConstants {
        public static final double kP = 0.0006;
        public static final double kI = 0.0;
        public static final double kD = 0.0;

        public static final double kS = 0.2; // Static friction voltage
        public static final double kV = 0.001764; // Velocity constant
        public static final double kA = 0.01; // Acceleration constant

        public static final double setVelocity = 3766; // Example set velocity in RPM
        public static final double speed = 0.1;
        public static final double feederspeed = 0.85;

        public static final double WHEEL_DIAMETER_METERS = 0.1016;
    }

    public static class intakeConstants {
        public static final double INTAKE_SPEED = 0.5;
        public static final double OUTTAKE_SPEED = -0.4;
        public static final double EXTENDING_SPEED = -0.1;
        public static final double RETRACTING_SPEED = 0.2;

        public static final double kArmP = 0.0;
        public static final double kArmI = 0.0;
        public static final double kArmD = 0.0;
        public static final double kArmS = 0.0;
        public static final double kArmA = 0.0;
        public static final double kArmV = 0.0;
        public static final double kArmG = 0.0;

        public static final double kRollP = 0.0;
        public static final double kRollI = 0.0;
        public static final double kRollD = 0.0;
        public static final double kRollS = 0.0;
        public static final double kRollA = 0.0;
        public static final double kRollV = 0.0;
        public static final double kRollG = 0.0;
    }

    public static class climbConstants {
        public static final double extendSpeed = 0.75;
        // Speed for declimbing
        public static final double declimbSpeed = 0.1;
        // Speed for contracting the arm and climbing
        public static final double contractSpeed = -0.75;
        // Position at which to stop motor

        public static final double maxPos = 780.0;
        public static final double minPos = 10.0;

    }

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
        public static final double LOOP_PERIOD_SECS = 0.02;

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

        // AdvantageKit simulation
        public static enum Mode {
            REAL, // Running on a real robot
            SIM, // Running a physics simulator
            REPLAY // Replaying from a log file
        }

        public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : Mode.SIM;;
    }

    /**
     * Field Constants
     */
    public static class FieldConstants {
        public static final Pose2d[][] SIMULATION_START_POSES = new Pose2d[][] {
                // Blue Alliance
                {
                        new Pose2d(3.500, 5.900, Rotation2d.fromDegrees(0.0)),
                        new Pose2d(3.500, 4.000, Rotation2d.fromDegrees(0.0)),
                        new Pose2d(3.500, 0.600, Rotation2d.fromDegrees(0.0))
                },
                // Red Alliance
                {
                        new Pose2d(13.000, 2.100, Rotation2d.fromDegrees(180.0)),
                        new Pose2d(13.000, 4.000, Rotation2d.fromDegrees(180.0)),
                        new Pose2d(13.000, 7.400, Rotation2d.fromDegrees(180.0))
                }
        };

        public static final Pose2d[] HUB_POSES = new Pose2d[] {
                new Pose2d(4.6256, 4.0345, Rotation2d.fromDegrees(0.0)),
                new Pose2d(11.9154, 4.0345, Rotation2d.fromDegrees(0.0))
        };

        public static final Pose2d[] BLUE_ZONE = new Pose2d[] {
                new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)),
                new Pose2d(4.6256, 8.0693, Rotation2d.fromDegrees(0.0))
        };

        public static final Pose2d[] NEUTRAL_ZONE = new Pose2d[] {
                new Pose2d(4.6256, 0.0, Rotation2d.fromDegrees(0.0)),
                new Pose2d(11.9154, 8.0693, Rotation2d.fromDegrees(0.0))
        };

        public static final Pose2d[] RED_ZONE = new Pose2d[] {
                new Pose2d(11.9154, 0.0, Rotation2d.fromDegrees(0.0)),
                new Pose2d(16.541, 8.0693, Rotation2d.fromDegrees(0.0))
        };
    }

    /**
     * Game Piece Constants
     */
    public static class FuelCellConstants {
        public static final double DIAMETER = .15; // meters
        public static final double MASS = 0.203; // kg could go to 0.227
    }

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
        public static final double maxLinearSpeed = 4.69;
        public static final double maxLinearAcceleration = 4.0;
        public static final double maxAngularAcceleration = 20.0;
        public static final double maxAngularSpeed = 8.0; // 4.69 / driveBaseRadius;

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

    /**
     * 
     */
    public static class TeleopConstants {
        public static final PathConstraints CONSTRAINTS = new PathConstraints(4.5, 4.0, Units.degreesToRadians(540),
                Units.degreesToRadians(720));

        public static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(
                SwerveDriveConstants.maxLinearSpeed,
                SwerveDriveConstants.maxLinearAcceleration);
        public static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(
                SwerveDriveConstants.maxLinearSpeed,
                SwerveDriveConstants.maxLinearAcceleration);
        public static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS = new TrapezoidProfile.Constraints(
                SwerveDriveConstants.maxAngularSpeed, SwerveDriveConstants.maxLinearAcceleration);
    }

    /**
     * 
     */
    public static class VisionConstants {
        // AprilTag layout
        public static AprilTagFieldLayout aprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

        // Camera names and positions, must match names configured on coprocessor
        public static Camera frontCamera = new Camera("FrontCam", "limelight-four",
                new Transform3d(0.2, 0.0, 0.2, new Rotation3d(0.0, -0.4, 0.0)));
        public static Camera rearCamera = new Camera("RearCam", "limelight-three",
                new Transform3d(-0.2, 0.0, 0.2, new Rotation3d(0.0, -0.4, Math.PI)));

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
