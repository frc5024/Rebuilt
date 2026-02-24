package frc.robot.mechanisms;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;

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
    public void update(double intakeArmAngle, double climbShaftLength, double shooterArmAngle,
            double hopperPosition) {
        robotMechanism.setMechanisms(intakeArmAngle, climbShaftLength, shooterArmAngle, hopperPosition);

        Logger.recordOutput("Mechanism3d/Measured",
                new Pose3d[] {
                        new Pose3d(0.29, 0.0, 0.21, new Rotation3d(0.0, robotMechanism.getIntakeArmAngle(), 0.0)), // intake
                        new Pose3d(-0.29, -0.05, robotMechanism.getClimbShaftLength(), new Rotation3d(0.0, 0.0, 0.0)), // climber
                        new Pose3d(-0.15, 0.16, 0.37, new Rotation3d(0.0, 0.0, robotMechanism.getShooterArmAngle())), // shooter
                        new Pose3d(0.09, 0.0, 0.02, new Rotation3d(0.0, 0.0, robotMechanism.getHopperArmAngle())) // spindexer
                });
    }

}
