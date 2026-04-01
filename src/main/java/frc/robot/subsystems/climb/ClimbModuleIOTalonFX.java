package frc.robot.subsystems.climb;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * 
 */
public class ClimbModuleIOTalonFX implements ClimbModuleIO {
    // Constants
    protected final double GEAR_RATIO = 4.0;

    // Hardware
    protected final TalonFX climbMotor;

    // Inputs from climb motor
    private final StatusSignal<Angle> position;
    private final StatusSignal<AngularVelocity> velocity;
    private final StatusSignal<Voltage> appliedVolts;
    private final StatusSignal<Current> current;

    // Connection debouncers
    private final Debouncer connectedDebouncer;

    /**
     * 
     */
    public ClimbModuleIOTalonFX() {
        this.climbMotor = new TalonFX(3);

        // Create motor status signals
        this.position = this.climbMotor.getPosition();
        this.velocity = this.climbMotor.getVelocity();
        this.appliedVolts = this.climbMotor.getMotorVoltage();
        this.current = this.climbMotor.getStatorCurrent();

        // Configure periodic frames
        BaseStatusSignal.setUpdateFrequencyForAll(100.0, this.position);
        BaseStatusSignal.setUpdateFrequencyForAll(
                50.0,
                this.velocity,
                this.appliedVolts,
                this.current);
        ParentDevice.optimizeBusUtilizationForAll(this.climbMotor);

        this.connectedDebouncer = new Debouncer(0.5);
    }

    @Override
    public void updateInputs(ClimbModuleIOInputs inputs) {
        if (DriverStation.isDisabled()) {
            stop();
        }

        // Refresh all signals
        StatusCode statusCode = BaseStatusSignal.refreshAll(position, velocity, appliedVolts, current);

        inputs.data = new ClimbModuleIOData(
                connectedDebouncer.calculate(statusCode.isOK()),
                Units.rotationsToRadians(position.getValueAsDouble()),
                Units.rotationsToRadians(velocity.getValueAsDouble()),
                appliedVolts.getValueAsDouble(),
                0.0,
                current.getValueAsDouble(),
                0.0);
    }

    @Override
    public double getCurrentDrawAmps() {
        return current.getValueAsDouble();
    }

    @Override
    public double getFFCharacterizationVelocity() {
        return velocity.getValueAsDouble();
    }

    @Override
    public double getPosition() {
        return position.getValueAsDouble();
    }

    @Override
    public void runCharacterization(double voltage) {
        double voltageRequest = MathUtil.clamp(voltage * 12, -12.0, 12.0);
        climbMotor.setVoltage(voltageRequest);
    }
}