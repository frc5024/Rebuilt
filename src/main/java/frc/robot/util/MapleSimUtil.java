package frc.robot.util;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.ArrayList;

import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.MapleSimConstants;

public class MapleSimUtil {
    public static int CAPACITY = 20;
    private static double FUEL_WIDTH = .15;

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
                    Meters.of(0.6858),
                    Meters.of(0.2),
                    IntakeSimulation.IntakeSide.FRONT,
                    CAPACITY);
        }

        return intakeSimulation;
    }

    /**
     * 
     */
    public static Pose3d[] getFuelPoses() {
        ArrayList<Pose3d> poses = new ArrayList<>();
        Pose2d robotPose = getSwerveDriveSimulation().getSimulatedDriveTrainPose();

        int index = 0;
        int size = 3;
        double pX = 0.0, pY = 0.0, pZ = 0.0;
        for (int z = 0; z < size; z++) {
            for (int y = 0; y < size; y++) {
                for (int x = 0; x < size; x++) {
                    if (index < getIntakeSimulation().getGamePiecesAmount()) {
                        pX = (-FUEL_WIDTH / 2) + (FUEL_WIDTH * x);
                        pY = -FUEL_WIDTH + (FUEL_WIDTH * y);
                        pZ = (FUEL_WIDTH * z) + FUEL_WIDTH;

                        Transform3d transform3d = new Transform3d(pX, pY, pZ, new Rotation3d());
                        Pose3d pose = new Pose3d(robotPose).transformBy(transform3d);
                        poses.add(pose);
                        index++;
                    } else {
                        return poses.toArray(new Pose3d[0]);
                    }
                }
            }
        }

        return poses.toArray(new Pose3d[0]);
    }

    /**
     * 
     */
    public static void ejectFuelFromRobot() {
        if (!getIntakeSimulation().obtainGamePieceFromIntake()) {
            return;
        }

        Pose2d robotPose = getSwerveDriveSimulation().getSimulatedDriveTrainPose();
        Transform3d fuelTransform = new Transform3d(-FUEL_WIDTH, FUEL_WIDTH * 2, FUEL_WIDTH * 3, new Rotation3d());
        Pose3d fuelPose = new Pose3d(robotPose).transformBy(fuelTransform);
        double angle = 45.0;

        RebuiltFuelOnFly rebuiltFuelOnFly = new RebuiltFuelOnFly(
                getSwerveDriveSimulation().getSimulatedDriveTrainPose().getTranslation(),
                new Translation2d(fuelPose.getX(), fuelPose.getY()),
                getSwerveDriveSimulation().getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                getSwerveDriveSimulation().getSimulatedDriveTrainPose().getRotation(),
                Meters.of(fuelPose.getZ()),
                MetersPerSecond.of(3),
                Degrees.of(angle));

        SimulatedArena.getInstance().addGamePieceProjectile(rebuiltFuelOnFly);
    }
}
