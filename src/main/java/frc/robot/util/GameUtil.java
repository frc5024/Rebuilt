package frc.robot.util;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

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
     * Determines if the alliance hub is active based on match time and game data.
     * Hub activation cycles through shifts in teleop based on autonomous outcome.
     * 
     * @return true if the hub is currently active, false otherwise
     */
    public static boolean isHubActive() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        // If we have no alliance, we cannot be enabled, therefore no hub.
        if (alliance.isEmpty()) {
            return false;
        }
        // Hub is always enabled in autonomous.
        if (DriverStation.isAutonomousEnabled()) {
            return true;
        }
        // At this point, if we're not teleop enabled, there is no hub.
        if (!DriverStation.isTeleopEnabled()) {
            return false;
        }

        // We're teleop enabled, compute.
        double matchTime = DriverStation.getMatchTime();
        String gameData = DriverStation.getGameSpecificMessage();
        // If we have no game data, we cannot compute, assume hub is active, as its
        // likely early in teleop.
        if (gameData.isEmpty()) {
            return true;
        }

        boolean redInactiveFirst = false;
        switch (gameData.charAt(0)) {
            case 'R' -> redInactiveFirst = true;
            case 'B' -> redInactiveFirst = false;
            default -> {
                // If we have invalid game data, assume hub is active.
                return true;
            }
        }

        // Shift was is active for blue if red won auto, or red if blue won auto.
        boolean shift1Active = switch (alliance.get()) {
            case Red -> !redInactiveFirst;
            case Blue -> redInactiveFirst;
        };

        if (matchTime > 130) {
            // Transition shift, hub is active.
            return true;
        } else if (matchTime > 105) {
            // Shift 1
            return shift1Active;
        } else if (matchTime > 80) {
            // Shift 2
            return !shift1Active;
        } else if (matchTime > 55) {
            // Shift 3
            return shift1Active;
        } else if (matchTime > 30) {
            // Shift 4
            return !shift1Active;
        } else {
            // End game, hub always active.
            return true;
        }

        // For now, just return true - the actual phase timing needs to match simulator
        // return true;
    }

    /**
     * Determines if the alliance hub is inactive (opposite of isHubActive).
     * 
     * @return true if the hub is currently inactive, false if active
     */
    public static boolean isHubInactive() {
        return !isHubActive();
    }

    /**
     * Gets the time remaining in the current phase countdown.
     * Shows how much time is left within each phase of the match.
     * 
     * AUTO: Counts down from 20 to 0
     * TELEOP:
     * - Transition Shift (2:20-2:10): Counts down from 10 to 0
     * - Shift 1 (2:10-1:45): Counts down from 25 to 0
     * - Shift 2 (1:45-1:20): Counts down from 25 to 0
     * - Shift 3 (1:20-0:55): Counts down from 25 to 0
     * - Shift 4 (0:55-0:30): Counts down from 25 to 0
     * - End Game (0:30-0:00): Counts down from 30 to 0
     * 
     * @return time remaining in current phase (duration down to 0), or 0 if not in
     *         match
     */

    public static boolean wonAuto() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        String gameData = DriverStation.getGameSpecificMessage();

        if (alliance.isEmpty() || gameData.isEmpty()) {
            // Dashboard should see 'false', until we know for sure who won auto
            return false;
        }

        // Shift was is active for blue if red won auto, or red if blue won auto.
        boolean redInactiveFirst;
        switch (gameData.charAt(0)) {
            case 'R' -> redInactiveFirst = true;
            case 'B' -> redInactiveFirst = false;
            default -> {
                return false;
            }
        }

        if (alliance.get() == Alliance.Red) {
            return redInactiveFirst;
        } else {
            return !redInactiveFirst;
        }
    }

    public static double getTimeRemainingInPhase() {
        // In autonomous - count down from 20 to 0
        if (DriverStation.isAutonomousEnabled()) {
            double matchTime = DriverStation.getMatchTime();
            return Math.floor(matchTime); // AUTO is 20 seconds, so matchTime goes 20→0
        }

        // In teleop - calculate which phase and countdown for that phase
        if (DriverStation.isTeleopEnabled()) {
            double matchTime = DriverStation.getMatchTime();

            // Match time values: 2:20 = 140, 2:10 = 130, 1:45 = 105, 1:20 = 80, 0:55 = 55,
            // 0:30 = 30
            if (!wonAuto()) {
                if (matchTime > 105) {
                    // Transition Shift and Shift 1 (140 to 105) - 35 seconds
                    return Math.floor(matchTime - 105.0);
                } else if (matchTime > 80) {
                    // Shift 2 (105 to 80) - 25 seconds
                    return Math.floor(matchTime - 80.0);
                } else if (matchTime > 55) {
                    // Shift 3 (80 to 55) - 25 seconds
                    return Math.floor(matchTime - 55.0);
                } else if (matchTime > 30) {
                    // Shift 4 (55 to 30) - 25 seconds
                    return Math.floor(matchTime - 30.0);
                } else {
                    // End Game (30 to 0) - 30 seconds
                    return Math.floor(matchTime);
                }
            } else {
                if (matchTime > 130) {
                    // Transition Shift (140 to 130) - 10 seconds
                    return Math.floor(matchTime - 130.0);
                } else if (matchTime > 105) {
                    // Shift 1 (130 to 105) - 25 seconds
                    return Math.floor(matchTime - 105.0);
                } else if (matchTime > 80) {
                    // Shift 2 (105 to 80) - 25 seconds
                    return Math.floor(matchTime - 80.0);
                } else if (matchTime > 55) {
                    // Shift 3 (80 to 55) - 25 seconds
                    return Math.floor(matchTime - 55.0);
                } else {
                    // Shift 4 and End Game (55 to 0) - 55 seconds
                    return Math.floor(matchTime);
                }
            }
        }

        return 0.0; // Not in match
    }

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
        double vy = velocityMetersPerSec * Math.cos(-rotation3d.getY()) * Math.sin(rotation3d.getZ());
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
    public static Pose2d getTargetPose(Pose2d robotPose) {
        boolean isRedAlliance = DriverStation.getAlliance().get() == Alliance.Red;
        boolean isAboveMidLine = GameUtil.isAboveMidLine(robotPose);

        if (GameUtil.inAllianceZone(robotPose)) {
            return GameUtil.getHubPose();
        } else {
            return FieldConstants.MULE_POSES[isRedAlliance ? 1 : 0][isAboveMidLine ? 1 : 0];
        }
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
