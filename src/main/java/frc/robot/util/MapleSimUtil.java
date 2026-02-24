package frc.robot.util;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.MapleSimConstants;

public class MapleSimUtil {
    private static SwerveDriveSimulation swerveDriveSimulation;
    private static IntakeSimulation intakeSimulation;

    /**
     * 
     */
    public static SwerveDriveSimulation getSwerveDriveSimulation() {
        if (swerveDriveSimulation == null) {
            swerveDriveSimulation = new SwerveDriveSimulation(MapleSimConstants.mapleSimConfig,
                    new Pose2d(0, 0, new Rotation2d()));
        }

        return swerveDriveSimulation;
    }

    /**
     * 
     */
    public static IntakeSimulation getIntakeSimulation() {
        if (intakeSimulation == null) {
            intakeSimulation = IntakeSimulation.OverTheBumperIntake("Fuel", getSwerveDriveSimulation(),
                    Meters.of(0.5),
                    Meters.of(0.4), IntakeSimulation.IntakeSide.FRONT, 1);
        }

        return intakeSimulation;
    }

    /**
     * 
     */
    public static void ejectFuelFromRobot(Pose3d fuelPose) {
        Transform3d fuelTransform = new Transform3d(
                new Pose3d(getSwerveDriveSimulation().getSimulatedDriveTrainPose()), fuelPose);

        double angle = 45.0;

        SimulatedArena.getInstance()
                .addGamePieceProjectile(new ReefscapeCoralOnFly(
                        getSwerveDriveSimulation().getSimulatedDriveTrainPose().getTranslation(),
                        new Translation2d(fuelTransform.getX(), fuelTransform.getY()),
                        getSwerveDriveSimulation().getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                        getSwerveDriveSimulation().getSimulatedDriveTrainPose().getRotation(),
                        Meters.of(fuelTransform.getZ()),
                        MetersPerSecond.of(3),
                        Degrees.of(angle)));
    }
}
