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

    default double getGoalVelocity() {
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

    // used in tuning mode
    default void setFF(double kS, double kV, double kA) {
    }

    // used in tuning mode
    default void setPID(double kP, double kI, double kD) {
    }

    default void setVoltage(double targetRPM) {
    }

    default void stop() {
    }

    default void updateInputs(FeederModuleIOInputs inputs) {
    }
}
