package frc.robot.subsystems.turret;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;

/**
 * 
 */
public class TurretModuleIOSparkMax implements TurretModuleIO {
    protected final SparkMax turretMotor;
    private final RelativeEncoder encoder;

    // Connection debouncers
    private final Debouncer connectedDebouncer;

    /**
     * 
     */
    public TurretModuleIOSparkMax() {
        this.turretMotor = new SparkMax(Constants.turretConstants.turretMotorChannel,
                SparkLowLevel.MotorType.kBrushless);
        this.encoder = this.turretMotor.getEncoder();
        this.connectedDebouncer = new Debouncer(0.5);
    }

    @Override
    public void updateInputs(TurretModuleIOInputs inputs) {
        if (DriverStation.isDisabled()) {
            stop();
        }

        inputs.data = new TurretModuleIOData(
                connectedDebouncer.calculate(true), // TODO: add spark utility to test for connection
                encoder.getPosition(),
                encoder.getVelocity(),
                turretMotor.getAppliedOutput(),
                0.0,
                turretMotor.getOutputCurrent(),
                turretMotor.getMotorTemperature());
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
    public void set(double turretspeed) {
        turretMotor.set(-turretspeed);
    }

    @Override
    public void setPosition(double position) {
        encoder.setPosition(position);
    }

    @Override
    public void setVoltage(double voltage) {
        turretMotor.setVoltage(voltage);
    }
}
