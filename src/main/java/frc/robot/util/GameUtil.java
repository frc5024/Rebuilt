package frc.robot.util;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.FuelCellConstants;
import frc.robot.Constants.VisionConstants;

/**
 * Helpful functions when playing the game
 */
public class GameUtil {
    /**
     * 
     */
    public static boolean isAboveMidLine(Pose2d robotPose) {
        return robotPose.getY() > VisionConstants.aprilTagLayout.getFieldWidth() / 2;
    }

    /**
     * 
     */
    public static double getDistanceToHub(Pose2d robotPose) {
        Pose2d hubPose = getHubPose();

        return robotPose.getTranslation().getDistance(hubPose.getTranslation());
    }

    /**
     * 
     */
    public static Pose2d getHubPose() {
        return DriverStation.getAlliance().get() == Alliance.Blue ? FieldConstants.HUB_POSES[0]
                : FieldConstants.HUB_POSES[1];
    }

    /**
     * 
     */
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
    public static boolean inNeutralZone(Pose2d robotPose) {
        return robotPose.getX() >= FieldConstants.NEUTRAL_ZONE[0].getX() &&
                robotPose.getX() <= FieldConstants.NEUTRAL_ZONE[1].getX() &&
                robotPose.getY() >= FieldConstants.NEUTRAL_ZONE[0].getY() &&
                robotPose.getY() <= FieldConstants.NEUTRAL_ZONE[1].getY();
    }

    /**
     * 
     */
    public static boolean inAllianceZone(Pose2d robotPose) {
        Pose2d[] allianceZone = DriverStation.getAlliance().get() == Alliance.Blue ? FieldConstants.BLUE_ZONE
                : FieldConstants.RED_ZONE;

        return robotPose.getX() >= allianceZone[0].getX() &&
                robotPose.getX() <= allianceZone[1].getX() &&
                robotPose.getY() >= allianceZone[0].getY() &&
                robotPose.getY() <= allianceZone[1].getY();
    }
}
