package frc.robot.subsystems.intake;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.intakeConstants.ArmConstants;

/**
 * 
 */
public class ArmModuleIOSparkMax implements ArmModuleIO {
    // Constants
    protected final int MOTOR_ID = 500;
    protected final int RETRACTED_ID = 9;
    protected final int EXTENDED_ID = 10;

    protected final double GEAR_RATIO = 9.0;

    // Hardware
    protected final SparkMax armMotor;
    protected final RelativeEncoder armEncoder;
    private final SparkClosedLoopController pidController;

    private SparkMaxConfig config;

    protected final DigitalInput retractedLimit;
    protected final DigitalInput extendedLimit;

    // Connection debouncers
    private final Debouncer connectedDebouncer;

    /**
     * 
     */
    public ArmModuleIOSparkMax() {
        this.armMotor = new SparkMax(MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
        this.pidController = this.armMotor.getClosedLoopController();
        this.armEncoder = this.armMotor.getEncoder();

        this.config = new SparkMaxConfig();
        config.idleMode(SparkMaxConfig.IdleMode.kBrake);

        // set position factor so we can set arm to specific angle
        double positionFactor = 360.0 / GEAR_RATIO;
        config.encoder
                .positionConversionFactor(positionFactor)
                .velocityConversionFactor(positionFactor / 60.0);

        // set the arm limits by degree
        config.softLimit
                .reverseSoftLimitEnabled(true).reverseSoftLimit(ArmConstants.RETRACTED_ANGLE)
                .forwardSoftLimitEnabled(true).forwardSoftLimit(ArmConstants.EXTENDED_ANGLE);

        double[] kPIDs = ArmConstants.getPIDs();
        config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(kPIDs[0], kPIDs[1], kPIDs[2]);

        armMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        this.retractedLimit = new DigitalInput(RETRACTED_ID);
        this.extendedLimit = new DigitalInput(EXTENDED_ID);

        this.connectedDebouncer = new Debouncer(0.5);
    }

    @Override
    public void updateInputs(ArmModuleIOInputs inputs) {
        if (DriverStation.isDisabled()) {
            stop();
        }

        inputs.data = new ArmModuleIOData(
                connectedDebouncer.calculate(true),
                armEncoder.getPosition(),
                armEncoder.getVelocity(),
                armMotor.getAppliedOutput(),
                0.0,
                armMotor.getOutputCurrent(),
                armMotor.getMotorTemperature());
    }

    @Override
    public void extend() {
        pidController.setSetpoint(ArmConstants.EXTENDED_ANGLE, ControlType.kPosition);
    }

    @Override
    public double getCurrentDrawAmps() {
        return armMotor.getOutputCurrent();
    }

    @Override
    public double getGoalPosition() {
        return pidController.getSetpoint();
    }

    @Override
    public double getPosition() {
        return armEncoder.getPosition();
    }

    @Override
    public double getVelocity() {
        return armEncoder.getVelocity();
    }

    @Override
    public boolean isExtended() {
        return !extendedLimit.get();
    }

    @Override
    public boolean isRetracted() {
        return !retractedLimit.get();
    }

    @Override
    public void retract() {
        pidController.setSetpoint(ArmConstants.RETRACTED_ANGLE, ControlType.kPosition);
    }

    @Override
    public void setPID(double kP, double kI, double kD) {
        config.closedLoop.p(kP).i(kI).d(kD);

        armMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void setFF(double kS, double kV, double kA) {
        config.closedLoop.feedForward.kS(kS).kV(kV).kA(kA);

        armMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void setPosition(double degrees) {
        armEncoder.setPosition(degrees);
    }

    @Override
    public void stop() {
        armMotor.stopMotor();
    }
}
