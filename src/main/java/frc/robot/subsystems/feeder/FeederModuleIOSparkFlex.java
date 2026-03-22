package frc.robot.subsystems.feeder;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.FeederConstants;

/**
 * 
 */
public class FeederModuleIOSparkFlex implements FeederModuleIO {
    // Constants
    private final int MOTOR_ID = 6;
    protected final double GEAR_RATIO = 1.0;

    // Hardware
    protected final SparkFlex feederMotor;
    private final RelativeEncoder encoder;

    // Connection debouncers
    private final Debouncer connectedDebouncer;

    /**
     * 
     */
    public FeederModuleIOSparkFlex() {
        this.feederMotor = new SparkFlex(MOTOR_ID, MotorType.kBrushless);
        this.encoder = this.feederMotor.getEncoder();

        // Configure motor with current limit
        SparkFlexConfig config = new SparkFlexConfig();
        config
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(35)
                .openLoopRampRate(0.05)
                .inverted(true);
        feederMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

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
                encoder.getVelocity(),
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
    public void set(double feederspeed) {
        feederMotor.set(feederspeed);
    }

    @Override
    public void start() {
        feederMotor.set(FeederConstants.feederSpeed);
    }

    @Override
    public void stop() {
        feederMotor.stopMotor();
    }
}
