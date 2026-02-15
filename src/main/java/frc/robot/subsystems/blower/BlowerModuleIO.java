package frc.robot.subsystems.blower;

import org.littletonrobotics.junction.AutoLog;

/**
 * 
 */
public interface BlowerModuleIO {
    public static final double TARGET_VOLTAGE = 12.0;
    public static final double RAMP_TIME_SEC = 10.0;

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

    default boolean isRunning() {
        return false;
    }

    default void start() {
    }

    default void stop() {
    }

    default void updateInputs(BlowerModuleIOInputs inputs) {
    }
}
