package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

/**
 * 
 */
public interface RollerModuleIO {
    @AutoLog
    class RollerModuleIOInputs {
        public RollerModuleIOData data = new RollerModuleIOData(false, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    }

    record RollerModuleIOData(
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

    default double getFFCharacterizationVelocity() {
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

    default void runCharacterization(double voltage) {
    }

    default void setConstraints(double maxVelocity, double maxAcceleration, double tolerance) {
    }

    default void setFF(double kS, double kV, double kA) {
    }

    default void setPID(double kP, double kI, double kD) {
    }

    default void setVelocity(double rpm) {
    }

    default void stop() {
    }

    default void updateInputs(RollerModuleIOInputs inputs) {
    }
}
