package frc.robot.subsystems.shooter;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

/**
 * 
 */
public class ShooterModuleIOSparkFlex implements ShooterModuleIO {
    // Constants
    protected final double GEAR_RATIO = 1.0;

    // Hardware
    protected final SparkFlex flywheel1;
    private final SparkFlex flywheel2;

    private final SparkBaseConfig flywheel1MotorConfig = new SparkFlexConfig()
            .idleMode(IdleMode.kCoast) // sets the motors to coast mode
            .inverted(true);
    private final SparkBaseConfig flywheel2MotorConfig = new SparkFlexConfig()
            .idleMode(IdleMode.kCoast)
            .follow(51, true);

    private final RelativeEncoder flywheel1Encoder;
    private final RelativeEncoder flywheel2Encoder;

    // Connection debouncers
    private final Debouncer fw1ConnectedDebouncer;
    private final Debouncer fw2ConnectedDebouncer;

    ShuffleboardTab tab = Shuffleboard.getTab("Shooter");

    /**
     * 
     */
    public ShooterModuleIOSparkFlex() {
        this.flywheel1 = new SparkFlex(51, MotorType.kBrushless);
        this.flywheel2 = new SparkFlex(52, MotorType.kBrushless);

        flywheel1.configure(flywheel1MotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        flywheel2.configure(flywheel2MotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        this.flywheel1Encoder = this.flywheel1.getEncoder();
        this.flywheel2Encoder = this.flywheel2.getEncoder();

        this.fw1ConnectedDebouncer = new Debouncer(0.5);
        this.fw2ConnectedDebouncer = new Debouncer(0.5);

        tab.addDouble("Actual Velocity", () -> flywheel1Encoder.getVelocity());

    }

    @Override
    public void updateInputs(ShooterModuleIOInputs inputs) {
        if (DriverStation.isDisabled()) {
            stop();
        }

        inputs.data = new ShooterModuleIOData(
                fw1ConnectedDebouncer.calculate(true), // TODO: add spark utility to test for connection
                flywheel1Encoder.getPosition(),
                flywheel1Encoder.getVelocity(),
                flywheel1.getAppliedOutput(),
                flywheel1.getBusVoltage(),
                flywheel1.getOutputCurrent(),
                flywheel1.getMotorTemperature(),
                fw2ConnectedDebouncer.calculate(true), // TODO: add spark utility to test for connection
                flywheel2Encoder.getPosition(),
                flywheel2Encoder.getVelocity(),
                flywheel1.getAppliedOutput(),
                flywheel2.getBusVoltage(),
                flywheel2.getOutputCurrent(),
                flywheel2.getMotorTemperature());
    }

    @Override
    public double getPosition() {
        return this.flywheel1Encoder.getPosition();
    }

    @Override
    public double getVelocity() {
        return this.flywheel1Encoder.getVelocity();
    }

    @Override
    public void set(double speed) {
        this.flywheel1.set(speed);
    }

    @Override
    public void setVoltage(double voltage) {
        this.flywheel1.setVoltage(voltage);
    }
}
