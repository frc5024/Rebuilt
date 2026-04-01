package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

/**
 * 
 */
public interface ShooterModuleIO {
    @AutoLog
    class ShooterModuleIOInputs {
        public ShooterModuleIOData data = new ShooterModuleIOData(false, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, false, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0);
    }

    record ShooterModuleIOData(
            boolean fw1Connected,
            double fw1PositionRots,
            double fw1VelocityRPM,
            double fw1AppliedVoltage,
            double fw1BusVoltage,
            double fw1SupplyCurrentAmps,
            double fw1TempCelsius,
            boolean fw2Connected,
            double fw2PositionRots,
            double fw2VelocityRPM,
            double fw2AppliedVoltage,
            double fw2BusVoltage,
            double fw2SupplyCurrentAmps,
            double fw2TempCelsius) {
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

    default boolean isAtSetpoint() {
        return false;
    }

    default boolean isRunning() {
        return false;
    }

    default void runCharacterization(double voltage) {
    }

    // used in tuning mode
    default void setPID(double kP, double kI, double kD) {
    }

    // used in tuning mode
    default void setFF(double kS, double kV, double kA) {
    }

    default void setVelocity(double rpm) {
    }

    default void stop() {
    }

    default void updateInputs(ShooterModuleIOInputs inputs) {
    }
}
