package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.FieldConstants;

/**
 * Helpful functions when playing the game
 */
public class GameUtil {
    /**
     * 
     */
    public static double getDistanceToHub(Pose2d robotPose) {
        Pose2d hubPose = getHubPose();

        return Math.hypot(robotPose.getX() - hubPose.getX(), robotPose.getY() - hubPose.getY());
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
