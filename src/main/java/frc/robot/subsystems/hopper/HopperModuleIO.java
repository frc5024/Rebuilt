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

    default void updateInputs(HopperModuleIOInputs inputs) {
    }
}
