package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

/**
 * 
 */
public interface IntakeModuleIO {
    @AutoLog
    class IntakeModuleIOInputs {
        public IntakeModuleIOData data = new IntakeModuleIOData(false, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, false, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0);
    }

    record IntakeModuleIOData(
            boolean armConnected,
            double armPositionRads,
            double armVelocityRadsPerSec,
            double armAppliedVoltage,
            double armTorqueCurrentAmps,
            double armSupplyCurrentAmps,
            double armTempCelsius,
            boolean intakeConnected,
            double intakePositionRads,
            double intakeVelocityRadsPerSec,
            double intakeAppliedVoltage,
            double intakeTorqueCurrentAmps,
            double intakeSupplyCurrentAmps,
            double intakeTempCelsius) {
    }

    default double getPosition() {
        return 0.0;
    }

    default double getCurrentDrawAmps() {
        return 0.0;
    }

    default boolean isRunning() {
        return false;
    }

    // TODO: remove after refactoring - speed is a constant so set it in the
    // hardware module
    default void setIntake(double speed) {
    }

    // TODO: remove after refactoring - speed is a constant so set it in the
    // hardware module
    default void setArm(double speed) {
    }

    default boolean isIntakeRetracted() {
        return false;
    }

    default boolean isIntakeExtended() {
        return true;
    }

    default void start() {
    }

    default void stop() {
    }

    default void updateInputs(IntakeModuleIOInputs inputs) {
    }
}
