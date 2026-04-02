package frc.robot.subsystems.swervedrive;

import com.ctre.phoenix6.sim.Pigeon2SimState;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;

/**
 * 
 */
public class GyroModuleIOSim extends GyroModuleIOPigeon2 {
    // Hardware
    private final Pigeon2SimState pigeon2SimState;

    // Variables
    private double pitchRadians;
    private double rollRadians;
    private double yawRadians;

    /**
     * 
     */
    public GyroModuleIOSim() {
        this.pigeon2SimState = pigeon.getSimState();

        this.pitchRadians = 0.0;
        this.rollRadians = 0.0;
        this.yawRadians = 0.0;
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.data = new GyroModuleIOData(
                true,
                Rotation2d.fromRadians(pitchRadians),
                Units.radiansToDegrees(pitchRadians),
                0.0,
                Rotation2d.fromRadians(rollRadians),
                Units.radiansToDegrees(rollRadians),
                0.0,
                Rotation2d.fromRadians(yawRadians),
                Units.radiansToDegrees(yawRadians),
                0.0,
                new double[] { Timer.getFPGATimestamp() },
                new Rotation2d[] { Rotation2d.fromRadians(yawRadians) });
    }

    /**
     * 
     */
    public void setRawYaw(double yawRadians) {
        this.yawRadians = yawRadians;
        pigeon2SimState.setRawYaw(yawRadians);
    }
}
