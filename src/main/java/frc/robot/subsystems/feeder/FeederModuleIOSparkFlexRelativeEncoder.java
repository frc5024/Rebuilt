package frc.robot.subsystems.feeder;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.FeederConstants;

/**
 * 
 */
public class FeederModuleIOSparkFlexRelativeEncoder implements FeederModuleIO {
    // Constants
    private final int MOTOR_ID = 6;
    protected final double GEAR_RATIO = 9.0;

    // Hardware
    protected final SparkFlex feederMotor;
    private final RelativeEncoder encoder;

    // SVA & PID
    private final SimpleMotorFeedforward feedforward;
    private final PIDController pidController;

    // Connection debouncers
    private final Debouncer connectedDebouncer;

    /**
     * 
     */
    public FeederModuleIOSparkFlexRelativeEncoder() {
        this.feederMotor = new SparkFlex(MOTOR_ID, MotorType.kBrushless);
        this.encoder = this.feederMotor.getEncoder();

        // Configure motor with current limit
        SparkFlexConfig config = new SparkFlexConfig();
        config
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(35)
                .openLoopRampRate(0.05)
                .inverted(true);

        // set velocity factor
        double velocityConversionFactor = 1 / GEAR_RATIO;
        config.encoder
                .positionConversionFactor(velocityConversionFactor)
                .velocityConversionFactor(velocityConversionFactor);

        feederMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // get and set feedforward SVA constants
        double[] kSVAs = FeederConstants.getSVAs();
        this.feedforward = new SimpleMotorFeedforward(kSVAs[0], kSVAs[1]);

        // get and set constraints and PID constants
        double[] kPIDs = FeederConstants.getPIDs();
        this.pidController = new PIDController(kPIDs[0], kPIDs[1], kPIDs[2]);

        this.connectedDebouncer = new Debouncer(0.5);
    }

    @Override
    public void updateInputs(FeederModuleIOInputs inputs) {
        if (DriverStation.isDisabled()) {
            stop();
        }

        inputs.data = new FeederModuleIOData(
                connectedDebouncer.calculate(true),
                encoder.getPosition(),
                encoder.getVelocity() * GEAR_RATIO,
                feederMotor.getAppliedOutput(),
                0.0,
                feederMotor.getOutputCurrent(),
                feederMotor.getMotorTemperature());
    }

    @Override
    public double getCurrentDrawAmps() {
        return feederMotor.getOutputCurrent();
    }

    @Override
    public double getPosition() {
        return encoder.getPosition();
    }

    @Override
    public double getVelocity() {
        return encoder.getVelocity();
    }

    @Override
    public boolean isRunning() {
        return feederMotor.getAppliedOutput() != 0.0;
    }

    @Override
    public void setFF(double kS, double kV, double kA) {
        feedforward.setKs(kS);
        feedforward.setKv(kV);
        feedforward.setKa(kA);
    }

    @Override
    public void setPID(double kP, double kI, double kD) {
        pidController.setPID(kP, kI, kD);
    }

    @Override
    public void setVoltage(double targetRPM) {
        double currentRPM = encoder.getVelocity();

        double pidVoltage = pidController.calculate(currentRPM, targetRPM);
        double ffVoltage = feedforward.calculate(pidController.getSetpoint());

        double voltageRequest = MathUtil.clamp(ffVoltage + pidVoltage, -12.0, 12.0);

        if (targetRPM == 0.0) {
            feederMotor.setVoltage(0.0);
            pidController.reset();
        } else {
            feederMotor.setVoltage(voltageRequest);
        }

        Logger.recordOutput("Feeder/ffVoltage", ffVoltage);
        Logger.recordOutput("Feeder/pidVoltage", pidVoltage);
    }

    @Override
    public void stop() {
        feederMotor.stopMotor();
    }
}
