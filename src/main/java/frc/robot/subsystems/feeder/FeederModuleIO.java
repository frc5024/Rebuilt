package frc.robot.subsystems.feeder;

import org.littletonrobotics.junction.AutoLog;

/**
 * 
 */
public interface FeederModuleIO {
    @AutoLog
    class FeederModuleIOInputs {
        public FeederModuleIOData data = new FeederModuleIOData(false, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    }

    record FeederModuleIOData(
            boolean connected,
            double positionRots,
            double velocityRPM,
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

    default void updateInputs(FeederModuleIOInputs inputs) {
    }
}
