package frc.robot.subsystems.hopper;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.HopperConstants;

/**
 * 
 */
public class HopperModuleIOSparkMax implements HopperModuleIO {
    // Constants
    private final int MOTOR_ID = 8;
    protected final double GEAR_RATIO = 9.0;

    // Hardware
    protected final SparkMax hopperMotor;
    private final RelativeEncoder encoder;

    // Connection debouncers
    private final Debouncer connectedDebouncer;

    /**
     * 
     */
    public HopperModuleIOSparkMax() {
        this.hopperMotor = new SparkMax(MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
        this.encoder = this.hopperMotor.getEncoder();

        // Configure motor with current limit
        SparkMaxConfig config = new SparkMaxConfig();
        config
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(25)
                .openLoopRampRate(0.15)
                .inverted(true);
        hopperMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

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
                encoder.getVelocity(),
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
    public double getPosition() {
        return encoder.getPosition();
    }

    @Override
    public double getVelocity() {
        return encoder.getVelocity();
    }

    @Override
    public void set(double hopperspeed) {
        hopperMotor.set(hopperspeed);
    }

    @Override
    public void start() {
        hopperMotor.set(HopperConstants.hopperSpeed);
    }

    @Override
    public void stop() {
        hopperMotor.stopMotor();
    }
}
