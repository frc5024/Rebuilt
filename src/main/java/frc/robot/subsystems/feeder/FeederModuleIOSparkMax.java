package frc.robot.subsystems.feeder;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * 
 */
public class FeederModuleIOSparkMax implements FeederModuleIO {
    // Constants
    protected final double GEAR_RATIO = 1.0;

    // Hardware
    protected final SparkFlex feederMotor;
    private final RelativeEncoder encoder;

    // Connection debouncers
    private final Debouncer connectedDebouncer;

    /**
     * 
     */
    public FeederModuleIOSparkMax() {
        this.feederMotor = new SparkFlex(6, MotorType.kBrushless);

        // Configure motor with current limit
        SparkFlexConfig config = new SparkFlexConfig();
        feederMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        this.encoder = this.feederMotor.getEncoder();
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
    public void set(double feederspeed) {
        feederMotor.set(-feederspeed);
    }
}
