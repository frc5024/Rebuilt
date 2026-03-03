package frc.robot.subsystems.feeder;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * 
 */
public class FeederModuleIOSparkMax implements FeederModuleIO {
    // Constants
    protected final double GEAR_RATIO = 1.0;

    // Hardware
    protected final SparkMax feederMotor;
    private final RelativeEncoder encoder;

    // Connection debouncers
    private final Debouncer connectedDebouncer;

    /**
     * 
     */
    public FeederModuleIOSparkMax() {
        this.feederMotor = new SparkMax(6, MotorType.kBrushless);
        this.encoder = this.feederMotor.getEncoder();
        this.connectedDebouncer = new Debouncer(0.5);
    }

    @Override
    public void updateInputs(FeederModuleIOInputs inputs) {
        if (DriverStation.isDisabled()) {
            stop();
        }

        inputs.data = new FeederModuleIOData(
                connectedDebouncer.calculate(true), // TODO: add spark utility to test for connection
                encoder.getPosition(),
                encoder.getVelocity(),
                feederMotor.getAppliedOutput(),
                0.0,
                feederMotor.getOutputCurrent(),
                feederMotor.getMotorTemperature());
    }

    @Override
    public void set(double feederspeed) {
        this.feederMotor.set(-feederspeed);
    }
}
