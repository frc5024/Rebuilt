package frc.robot.subsystems.intake;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * 
 */
public class IntakeModuleIOSparkMax implements IntakeModuleIO {
    // Constants
    protected final int armMotorID = 5; // this is a placeholder ID
    protected final double GEAR_RATIO = 9.0;

    protected final int intakeMotorID = 60; // ID on prototype board, subject to change

    // Hardware
    protected final SparkMax armMotor;
    private final RelativeEncoder armEncoder;
    private final SparkBaseConfig armMotorConfig;

    protected final SparkFlex intakeMotor;
    private final RelativeEncoder intakeEncoder;
    private final SparkBaseConfig intakeMotorConfig;

    private static DigitalInput retractingLimitSwitch;
    private static DigitalInput extendingLimitSwitch;

    // Connection debouncers
    private final Debouncer connectedDebouncer;

    /**
     * 
     */
    public IntakeModuleIOSparkMax() {
        this.armMotor = new SparkMax(armMotorID, SparkLowLevel.MotorType.kBrushless);
        this.armEncoder = this.armMotor.getEncoder();
        this.armMotorConfig = new SparkMaxConfig();
        armMotorConfig
                .idleMode(IdleMode.kBrake);
        armMotor.configure(armMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        this.intakeMotor = new SparkFlex(intakeMotorID, SparkLowLevel.MotorType.kBrushless);
        this.intakeEncoder = this.intakeMotor.getEncoder();
        this.intakeMotorConfig = new SparkMaxConfig();
        intakeMotorConfig
                .idleMode(IdleMode.kCoast);
        intakeMotor.configure(intakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        retractingLimitSwitch = new DigitalInput(7);
        extendingLimitSwitch = new DigitalInput(8);

        this.connectedDebouncer = new Debouncer(0.5);
    }

    @Override
    public void updateInputs(IntakeModuleIOInputs inputs) {
        if (DriverStation.isDisabled()) {
            stop();
        }

        inputs.data = new IntakeModuleIOData(
                connectedDebouncer.calculate(true),
                armEncoder.getPosition(),
                armEncoder.getVelocity(),
                armMotor.getAppliedOutput(),
                0.0,
                armMotor.getOutputCurrent(),
                armMotor.getMotorTemperature(),
                connectedDebouncer.calculate(true),
                intakeEncoder.getPosition(),
                intakeEncoder.getVelocity(),
                intakeMotor.getAppliedOutput(),
                0.0,
                intakeMotor.getOutputCurrent(),
                intakeMotor.getMotorTemperature());
    }

    @Override
    public double getCurrentDrawAmps() {
        return armMotor.getOutputCurrent() + intakeMotor.getOutputCurrent();
    }

    @Override
    public double getArmPosition() {
        return armEncoder.getPosition();
    }

    @Override
    public double getArmVelocity() {
        return armEncoder.getVelocity();
    }

    @Override
    public double getIntakeVelocity() {
        return intakeEncoder.getVelocity();
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

    @Override
    public void setArm(double speed) {
        armMotor.set(speed);
    }

    @Override
    public void setIntake(double speed) {
        intakeMotor.set(speed);
    }
}
