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

    default void setAngle(double degrees) {
    }

    // used in tuning mode
    default void setConstraints(double maxVelocity, double maxAcceleration, double tolerance) {
    }

    // used in tuning mode
    default void setFF(double kS, double kV, double kA) {
    }

    // used in tuning mode
    default void setPID(double kP, double kI, double kD) {
    }

    default void setPosition(double position) {
    }

    default void stop() {
    }

    default void updateInputs(ArmModuleIOInputs inputs) {
    }
}
