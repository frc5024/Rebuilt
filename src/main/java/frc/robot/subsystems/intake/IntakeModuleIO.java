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
            double armPositionRots,
            double armVelocityRPM,
            double armAppliedVoltage,
            double armTorqueCurrentAmps,
            double armSupplyCurrentAmps,
            double armTempCelsius,
            boolean intakeConnected,
            double intakePositionRots,
            double intakeVelocityRPM,
            double intakeAppliedVoltage,
            double intakeTorqueCurrentAmps,
            double intakeSupplyCurrentAmps,
            double intakeTempCelsius) {
    }

    default double getCurrentDrawAmps() {
        return 0.0;
    }

    default double getArmPosition() {
        return 0.0;
    }

    default double getArmVelocity() {
        return 0.0;
    }

    default double getIntakeVelocity() {
        return 0.0;
    }

    default boolean isRunning() {
        return false;
    }

    default boolean isIntakeExtended() {
        return false;
    }

    default boolean isIntakeIntaking() {
        return false;
    }

    default boolean isIntakeRetracted() {
        return true;
    }

    default void setArm(double speed) {
    }

    default void setIntake(double speed) {
    }

    default void setIntakeEncoderPosition(double position) {

    }

    default void start() {
    }

    default void stop() {
    }

    default void updateInputs(IntakeModuleIOInputs inputs) {
    }
}
