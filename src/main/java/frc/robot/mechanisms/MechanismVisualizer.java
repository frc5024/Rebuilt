package frc.robot.mechanisms;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.util.GameUtil;

/**
 * 
 */
public class MechanismVisualizer {
    private final RobotMechanism robotMechanism;

    /**
     * 
     */
    public MechanismVisualizer() {
        this.robotMechanism = new RobotMechanism();
    }

    /**
     * 
     */
    public void update(double intakeArmPosition, double hopperPosition, double turretArmAngle, double climbShaftLength,
            double feederArmPosition, Pose3d turretPose, double shooterVelocity, Rotation2d[] wheelAngles) {
        robotMechanism.setMechanisms(intakeArmPosition, hopperPosition, turretArmAngle, climbShaftLength,
                feederArmPosition);

        Logger.recordOutput("Mechanism/2d", robotMechanism.getMechanism());

        Logger.recordOutput("Mechanism/3d",
                new Pose3d[] {
                        new Pose3d(0.29, 0.0, 0.21, new Rotation3d(0.0, intakeArmPosition, 0.0)), // intake
                        new Pose3d(-0.29, -0.05, robotMechanism.getClimbShaftLength(), new Rotation3d(0.0, 0.0, 0.0)), // climber
                        new Pose3d(-0.15, 0.16, 0.37,
                                new Rotation3d(0.0, 0.0, Units.degreesToRadians(robotMechanism.getTurretArmAngle()))), // turret
                        new Pose3d(0.09, 0.0, 0.055, new Rotation3d(0.0, 0.0, robotMechanism.getHopperArmAngle())), // spindexer
                        new Pose3d(-0.09, 0.0, 0.275, new Rotation3d(0.0, robotMechanism.getFeederArmAngle(), 0.0)), // feeder

                        new Pose3d(0.27, 0.27, 0.02,
                                new Rotation3d(0.0, 0.0, wheelAngles[0].getRadians())), // FL0
                        new Pose3d(-0.27, 0.27, 0.02,
                                new Rotation3d(0.0, 0.0, wheelAngles[2].getRadians())), // BL2
                        new Pose3d(-0.27, -0.27, 0.02,
                                new Rotation3d(0.0, 0.0, wheelAngles[3].getRadians())), // BR3
                        new Pose3d(0.27, -0.27, 0.02,
                                new Rotation3d(0.0, 0.0, wheelAngles[1].getRadians())), // FR1
                });

        Logger.recordOutput("Mechanism/Trajectory", GameUtil.getShooterTrajectory(turretPose, shooterVelocity));
    }

}
