package frc.robot.util;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.ArrayList;
import java.util.List;

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
import frc.robot.Constants.FuelCellConstants;
import frc.robot.Constants.HopperConstants;
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
                    Meters.of(0.6858),
                    Meters.of(0.2),
                    IntakeSimulation.IntakeSide.FRONT,
                    HopperConstants.CAPACITY);
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
                        pX = (-FuelCellConstants.DIAMETER / 2) + (FuelCellConstants.DIAMETER * x);
                        pY = -FuelCellConstants.DIAMETER + (FuelCellConstants.DIAMETER * y);
                        pZ = (FuelCellConstants.DIAMETER * z) + FuelCellConstants.DIAMETER;

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

    public static Pose3d[] getShooterTrajectory(Pose3d turretPose, double velocityMetersPerSec) {
        List<Pose3d> trajectory = new ArrayList<>();

        // Physical Constants for 2026 Fuel (Approximated)
        final double mass = FuelCellConstants.MASS;
        final double radius = FuelCellConstants.DIAMETER / 2; // meters
        final double area = Math.PI * Math.pow(radius, 2);
        final double dragCoeff = 0.47; // Sphere drag coefficient
        final double rho = 1.225; // Air density at sea level (kg/m^3)
        final double gravity = -9.81;

        // Initial State
        double x = turretPose.getX();
        double y = turretPose.getY();
        double z = turretPose.getZ();

        Rotation3d rotation3d = turretPose.getRotation();
        double vx = velocityMetersPerSec * Math.cos(rotation3d.getY()) * Math.cos(rotation3d.getZ());
        double vy = velocityMetersPerSec * Math.cos(rotation3d.getY()) * Math.sin(rotation3d.getZ());
        double vz = velocityMetersPerSec * Math.sin(-rotation3d.getY());

        double dt = 0.02; // 20ms steps for higher precision

        for (double t = 0; t < 2.0; t += dt) {
            double v = Math.sqrt(vx * vx + vy * vy + vz * vz);

            // Calculate Drag Acceleration magnitude
            double aDrag = (0.5 * rho * v * v * dragCoeff * area) / mass;

            // Apply drag acceleration against the velocity vector
            vx -= (aDrag * (vx / v)) * dt;
            vy -= (aDrag * (vy / v)) * dt;
            vz -= (aDrag * (vz / v)) * dt;

            // Apply Gravity
            vz += gravity * dt;

            // Update Positions
            x += vx * dt;
            y += vy * dt;
            z += vz * dt;

            trajectory.add(new Pose3d(x, y, z, new Rotation3d()));

            if (z <= 0)
                break; // Ground hit
        }

        return trajectory.toArray(new Pose3d[0]);
    }

    /**
     * 
     */
    public static void ejectFuelFromRobot() {
        if (!getIntakeSimulation().obtainGamePieceFromIntake()) {
            return;
        }

        Pose2d robotPose = getSwerveDriveSimulation().getSimulatedDriveTrainPose();
        Transform3d fuelTransform = new Transform3d(-FuelCellConstants.DIAMETER, FuelCellConstants.DIAMETER * 2,
                FuelCellConstants.DIAMETER * 3,
                new Rotation3d());
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
