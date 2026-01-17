package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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

    public static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(maxLinearSpeed,
            maxLinearAcceleration);
    public static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(maxLinearSpeed,
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
        private static final double DRIVE_KV_ROT = 0.91035; // Same units as TunerConstants: (volt * secs) / rotation
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
                                KilogramSquareMeters.of(TunerConstants.FrontLeft.SteerInertia),
                                RobotConstants.WHEEL_COF));
    }

    /**
     * 
     */
    public static class SwerveDriveConstants {
        /** Returns an array of module translations. */
        public static Translation2d[] getModuleTranslations() {
            return new Translation2d[] {
                    new Translation2d(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
                    new Translation2d(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY),
                    new Translation2d(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
                    new Translation2d(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)
            };
        }
    }
}
