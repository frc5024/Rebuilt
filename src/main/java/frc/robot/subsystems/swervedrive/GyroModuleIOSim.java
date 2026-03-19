package frc.robot.subsystems.swervedrive;

import com.ctre.phoenix6.sim.Pigeon2SimState;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;

/**
 * 
 */
public class GyroModuleIOSim extends GyroModuleIOPigeon2 {
    // Hardware
    private final Pigeon2SimState pigeon2SimState;

    // Variables
    private double yawRadians;

    /**
     * 
     */
    public GyroModuleIOSim() {
        this.pigeon2SimState = pigeon.getSimState();
        this.yawRadians = 0.0;
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = true;
        inputs.yawPosition = Rotation2d.fromRadians(yawRadians);
        inputs.yawVelocityRadPerSec = 0.0;
        inputs.odometryYawTimestamps = new double[] { Timer.getFPGATimestamp() };
        inputs.odometryYawPositions = new Rotation2d[] { Rotation2d.fromRadians(yawRadians) };
    }

    /**
     * 
     */
    public void setRawYaw(double yawRadians) {
        this.yawRadians = yawRadians;
        pigeon2SimState.setRawYaw(yawRadians);
    }
}
