package frc.robot.subsystems.blower;

import org.littletonrobotics.junction.AutoLog;

/**
 * 
 */
public interface BlowerModuleIO {
    @AutoLog
    class BlowerModuleIOInputs {
        public BlowerModuleIOData data = new BlowerModuleIOData(false, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    }

    record BlowerModuleIOData(
            boolean connected,
            double positionRads,
            double velocityRadsPerSec,
            double appliedVoltage,
            double torqueCurrentAmps,
            double supplyCurrentAmps,
            double tempCelsius) {
    }

    default void updateInputs(BlowerModuleIOInputs inputs) {
    }

    default void start(double speed) {
    }

    default void stop() {
    }
}
