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
    protected final double GEAR_RATIO = 9.0;
    protected final int intakeMotorID = 60; // ID on prototype board, subject to change
    protected final int armMotorID = 5; // this is a placeholder ID

    // Hardware
    protected final SparkMax armMotor;
    private final RelativeEncoder armEncoder;

    protected final SparkFlex intakeMotor;
    private final RelativeEncoder intakeEncoder;

    private static DigitalInput retractingLimitSwitch = new DigitalInput(7);
    private static DigitalInput extendingLimitSwitch = new DigitalInput(8);

    // Connection debouncers
    private final Debouncer connectedDebouncer;

    /**
     * 
     */
    public IntakeModuleIOSparkMax() {
        this.armMotor = new SparkMax(armMotorID, SparkLowLevel.MotorType.kBrushless);
        this.armEncoder = this.armMotor.getEncoder();
        SparkBaseConfig armMotorConfig = new SparkMaxConfig();
        armMotorConfig
                .idleMode(IdleMode.kBrake);
        this.armMotor.configure(armMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        this.intakeMotor = new SparkFlex(intakeMotorID, SparkLowLevel.MotorType.kBrushless);
        this.intakeEncoder = this.intakeMotor.getEncoder();
        SparkBaseConfig intakeMotorConfig = new SparkMaxConfig();
        intakeMotorConfig
                .idleMode(IdleMode.kCoast);
        this.intakeMotor.configure(intakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

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
    public double getIntakeVelocity() {
        return intakeMotor.getEncoder().getVelocity();
    }

    @Override
    public double getArmVelocity() {
        return armMotor.getEncoder().getVelocity();
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
