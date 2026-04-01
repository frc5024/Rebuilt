package frc.robot.subsystems.hopper;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.HopperConstants;

/**
 * 
 */
public class HopperModuleIOSparkMaxRelativeEncoder implements HopperModuleIO {
    // Constants
    private final int MOTOR_ID = 8;
    protected final double GEAR_RATIO = 9.0;

    // Hardware
    protected final SparkMax hopperMotor;
    private final RelativeEncoder encoder;

    // SVA & PID
    private final SimpleMotorFeedforward feedforward;
    private final PIDController pidController;

    // Connection debouncers
    private final Debouncer connectedDebouncer;

    /**
     * 
     */
    public HopperModuleIOSparkMaxRelativeEncoder() {
        this.hopperMotor = new SparkMax(MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
        this.encoder = this.hopperMotor.getEncoder();

        // Configure motor with current limit
        SparkMaxConfig config = new SparkMaxConfig();
        config
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(30)
                .openLoopRampRate(0.15)
                .inverted(false);

        // set velocity factor
        double velocityConversionFactor = 1 / GEAR_RATIO;
        config.encoder
                .positionConversionFactor(velocityConversionFactor)
                .velocityConversionFactor(velocityConversionFactor);

        hopperMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // get and set feedforward SVA constants
        double[] kSVAs = HopperConstants.getSVAs();
        this.feedforward = new SimpleMotorFeedforward(kSVAs[0], kSVAs[1]);

        // get and set constraints and PID constants
        double[] kPIDs = HopperConstants.getPIDs();
        this.pidController = new PIDController(kPIDs[0], kPIDs[1], kPIDs[2]);

        this.connectedDebouncer = new Debouncer(0.5);
    }

    @Override
    public void updateInputs(HopperModuleIOInputs inputs) {
        if (DriverStation.isDisabled()) {
            stop();
        }

        inputs.data = new HopperModuleIOData(
                connectedDebouncer.calculate(true),
                encoder.getPosition(),
                encoder.getVelocity() * GEAR_RATIO,
                hopperMotor.getAppliedOutput(),
                0.0,
                hopperMotor.getOutputCurrent(),
                hopperMotor.getMotorTemperature());
    }

    @Override
    public double getCurrentDrawAmps() {
        return hopperMotor.getOutputCurrent();
    }

    @Override
    public double getFFCharacterizationVelocity() {
        return encoder.getVelocity();
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
        return hopperMotor.getAppliedOutput() != 0.0;
    }

    @Override
    public void runCharacterization(double voltage) {
        double voltageRequest = MathUtil.clamp(voltage * 12, -12.0, 12.0);
        hopperMotor.setVoltage(voltageRequest);
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
            hopperMotor.setVoltage(0.0);
            pidController.reset();
        } else {
            hopperMotor.setVoltage(voltageRequest);
        }

        Logger.recordOutput("Hopper/ffVoltage", ffVoltage);
        Logger.recordOutput("Hopper/pidVoltage", pidVoltage);
    }

    @Override
    public void stop() {
        hopperMotor.stopMotor();
    }
}
