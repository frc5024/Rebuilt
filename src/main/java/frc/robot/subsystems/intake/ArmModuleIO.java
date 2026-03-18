package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

/**
 * 
 */
public interface ArmModuleIO {
    @AutoLog
    class ArmModuleIOInputs {
        public ArmModuleIOData data = new ArmModuleIOData(false, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    }

    record ArmModuleIOData(
            boolean connected,
            double positionRads,
            double velocityRadsPerSec,
            double appliedVoltage,
            double torqueCurrentAmps,
            double supplyCurrentAmps,
            double tempCelsius) {
    }

    default void extend() {
    }

    default double getCurrentDrawAmps() {
        return 0.0;
    }

    default double getGoalPosition() {
        return 0.0;
    }

    default double getPosition() {
        return 0.0;
    }

    default double getVelocity() {
        return 0.0;
    }

    default boolean isExtended() {
        return false;
    }

    default boolean isRetracted() {
        return true;
    }

    default void retract() {
    }

    default void setFF(double kS, double kV, double kA) {
    }

    default void setPID(double kP, double kI, double kD) {
    }

    default void setPosition(double degrees) {
    }

    default void start() {
    }

    default void stop() {
    }

    default void updateInputs(ArmModuleIOInputs inputs) {
    }
}
