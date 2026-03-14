package frc.robot.subsystems.hopper;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;

/**
 * 
 */
public class HopperModuleIOSparkMax implements HopperModuleIO {
    // Constants
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
        this.hopperMotor = new SparkMax(Constants.HopperConstants.HopperMotorID, SparkLowLevel.MotorType.kBrushless);
        this.encoder = this.hopperMotor.getEncoder();
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
    public void set(double hopperspeed) {
        this.hopperMotor.set(-hopperspeed);
    }
}
