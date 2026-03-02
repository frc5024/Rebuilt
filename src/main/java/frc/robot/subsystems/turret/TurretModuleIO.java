package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.AutoLog;

/**
 * 
 */
public interface TurretModuleIO {
    @AutoLog
    class TurretModuleIOInputs {
        public TurretModuleIOData data = new TurretModuleIOData(false, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    }

    record TurretModuleIOData(
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

    default double getVelocity() {
        return 0.0;
    }

    default boolean isRunning() {
        return false;
    }

    // TODO: remove after refactoring - speed is a constant so set it in the
    // hardware module
    default void set(double speed) {
    }

    default void setPosition(double position) {
    }

    default void setVoltage(double voltage) {
    }

    default void start() {
    }

    default void stop() {
    }

    default void updateInputs(TurretModuleIOInputs inputs) {
    }
}
