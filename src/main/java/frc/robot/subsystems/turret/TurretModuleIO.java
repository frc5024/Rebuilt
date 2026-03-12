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

    default boolean atGoal() {
        return false;
    }

    default double getCurrentDrawAmps() {
        return 0.0;
    }

    default double getCurrentAngle() {
        return 0.0;
    }

    default double getGoalPosition() {
        return 0.0;
    }

    default double getGoalVelocity() {
        return 0.0;
    }

    default boolean getHallEffect() {
        return false;
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

    default void set(double speed) {
    }

    default void setAngle(double degrees) {
    }

    default void setConstraints(double maxVelocity, double maxAcceleration, double tolerance) {
    }

    default void setFF(double kS, double kV, double kA) {
    }

    default void setPID(double kP, double kI, double kD) {
    }

    default void setPosition(double position) {
    }

    default void setVoltage() {
    }

    default void start() {
    }

    default void stop() {
    }

    default void updateInputs(TurretModuleIOInputs inputs) {
    }
}
