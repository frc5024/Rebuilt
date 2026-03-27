package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.AutoLog;

/**
 * 
 */
public interface ClimbModuleIO {
    @AutoLog
    class ClimbModuleIOInputs {
        public ClimbModuleIOData data = new ClimbModuleIOData(false, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    }

    record ClimbModuleIOData(
            boolean connected,
            double positionRads,
            double velocityRadsPerSec,
            double appliedVoltage,
            double torqueCurrentAmps,
            double supplyCurrentAmps,
            double tempCelsius) {
    }

    default double getCurrentDrawAmps() {
        return 0.0;
    }

    default double getPosition() {
        return 0.0;
    }

    public void zeroPosition();

    default boolean isRunning() {
        return false;
    }

    // TODO: remove after refactoring - speed is a constant so set it in the
    // hardware module
    default void set(double speed) {
    }

    default void start() {
    }

    default void stop() {
    }

    default void updateInputs(ClimbModuleIOInputs inputs) {
    }
}
