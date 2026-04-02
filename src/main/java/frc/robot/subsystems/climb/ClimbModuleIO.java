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
            double positionRots,
            double velocityRotsPerSec,
            double appliedVoltage,
            double torqueCurrentAmps,
            double supplyCurrentAmps,
            double tempCelsius) {
    }

    default double getCurrentDrawAmps() {
        return 0.0;
    }

    default double getFFCharacterizationVelocity() {
        return 0.0;
    }

    default double getLinearDistanceInches() {
        return 0.0;
    }

    default double getPosition() {
        return 0.0;
    }

    default double getVelocity() {
        return 0.0;
    }

    default void holdPosition() {
    }

    default boolean isSuspended() {
        return false;
    }

    default void runCharacterization(double voltage) {
    }

    // used in tuning mode
    default void setFF(double kS, double kV, double kA) {
    }

    // used in tuning mode
    default void setPID(double kP, double kI, double kD) {
    }

    default void setPosition(double inches, boolean isHooked) {
    }

    default void setVoltage(double targetRPM) {
    }

    default void start() {
    }

    default void stop() {
    }

    default void updateInputs(ClimbModuleIOInputs inputs) {
    }

    default public void zeroPosition() {
    }
}
