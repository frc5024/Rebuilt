package frc.robot.subsystems.intake;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * 
 */
public class IntakeModuleIOSparkMax implements IntakeModuleIO {
    int intakeMotorID = 60; // ID on prototype board, subject to change
    int armMotorID = 5; // this is a placeholder ID

    protected final SparkMax intakeMotor;
    protected final SparkMax armMotor;
    private final RelativeEncoder intakeEncoder;
    private final RelativeEncoder armEncoder;

    private static DigitalInput retractingLimitSwitch = new DigitalInput(7);
    private static DigitalInput extendingLimitSwitch = new DigitalInput(8);

    // Connection debouncers
    private final Debouncer connectedDebouncer;

    /**
     * 
     */
    public IntakeModuleIOSparkMax() {
        this.intakeMotor = new SparkMax(intakeMotorID, SparkLowLevel.MotorType.kBrushless);
        this.armMotor = new SparkMax(armMotorID, SparkLowLevel.MotorType.kBrushless);
        this.intakeEncoder = this.intakeMotor.getEncoder();
        this.armEncoder = this.armMotor.getEncoder();
        this.connectedDebouncer = new Debouncer(0.5);
    }

    @Override
    public void updateInputs(IntakeModuleIOInputs inputs) {
        if (DriverStation.isDisabled()) {
            stop();
        }

        inputs.data = new IntakeModuleIOData(
                connectedDebouncer.calculate(true), // TODO: add spark utility to test for connection
                armEncoder.getPosition(),
                armEncoder.getVelocity(),
                armMotor.getAppliedOutput(),
                0.0,
                armMotor.getOutputCurrent(),
                armMotor.getMotorTemperature(),
                connectedDebouncer.calculate(true), // TODO: add spark utility to test for connection
                intakeEncoder.getPosition(),
                intakeEncoder.getVelocity(),
                intakeMotor.getAppliedOutput(),
                0.0,
                intakeMotor.getOutputCurrent(),
                intakeMotor.getMotorTemperature());
    }

    @Override
    public double getPosition() {
        return armEncoder.getPosition();
    }

    @Override
    public void setArm(double speed) {
        armMotor.set(speed);
    }

    @Override
    public void setIntake(double speed) {
        intakeMotor.set(speed);
    }

    @Override
    public boolean isIntakeExtended() {
        return !extendingLimitSwitch.get();
    }

    @Override
    public boolean isIntakeIntaking() {
        return intakeEncoder.getVelocity() > 0.0;
    }

    @Override
    public boolean isIntakeRetracted() {
        return !retractingLimitSwitch.get();
    }
}
