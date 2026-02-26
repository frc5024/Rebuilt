package frc.robot.subsystems.hopper;

import org.littletonrobotics.junction.AutoLog;

/**
 * 
 */
public interface HopperModuleIO {
    @AutoLog
    class HopperModuleIOInputs {
        public HopperModuleIOData data = new HopperModuleIOData(false, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    }

    record HopperModuleIOData(
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

    default void updateInputs(HopperModuleIOInputs inputs) {
    }
}
