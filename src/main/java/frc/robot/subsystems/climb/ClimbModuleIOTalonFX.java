package frc.robot.subsystems.climb;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.ClimbConstants;

/**
 * 
 */
public class ClimbModuleIOTalonFX implements ClimbModuleIO {
    // Constants
    protected final int MOTOR_ID = 3;
    protected final double GEAR_RATIO = 40.0;
    protected final double DRUM_RADIUS = Units.inchesToMeters(0.75); // 1.5" diameter
    protected final double INCHES_PER_ROTATION = Units.metersToInches(2.0 * DRUM_RADIUS * Math.PI); // Circumference of
                                                                                                    // your winch spool

    // Hardware
    protected final TalonFX climbMotor;
    protected final TalonFXConfiguration config;
    private final MotionMagicVoltage motionMagicRequest;

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
        this.climbMotor = new TalonFX(MOTOR_ID);
        this.motionMagicRequest = new MotionMagicVoltage(0.0);

        // Configure the motor
        this.config = new TalonFXConfiguration();

        config.Feedback.SensorToMechanismRatio = GEAR_RATIO;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 15.0; // max height extension
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0; // don't smash the bottom

        double[] kSVAs = ClimbConstants.getSVAs();
        double[] kPIDs = ClimbConstants.getPIDs();

        // Slot 0: moving climb shaft when unweighted (not on the bar)
        config.Slot0.kS = kSVAs[0];
        config.Slot0.kV = kSVAs[1];
        config.Slot0.kA = kSVAs[2];
        // config.Slot1.kG = 0.05; // voltage needed to keep shaft at height

        config.Slot0.kP = kPIDs[0]; // higher du to lifing robot weight
        config.Slot0.kI = kPIDs[1];
        config.Slot0.kD = kPIDs[2];

        // Slot 1: moving climb shaft when weighted (trying to climb)
        config.Slot1.kG = 0.8; // gravity offset - voltage needed to hold robot in air
        config.Slot1.kP = 20.0;

        config.MotionMagic.MotionMagicCruiseVelocity = 40; // rotations per second
        config.MotionMagic.MotionMagicAcceleration = 80; // rotations per second squared

        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 60; // amps

        this.climbMotor.getConfigurator().apply(config);

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
    public double getLinearDistanceInches() {
        return position.getValueAsDouble() * INCHES_PER_ROTATION;
    }

    @Override
    public double getPosition() {
        return position.getValueAsDouble();
    }

    @Override
    public void holdPosition() {
        MotionMagicVoltage request = motionMagicRequest.withPosition(position.getValueAsDouble());
        climbMotor.setControl(request);
    }

    @Override
    public boolean isSuspended() {
        return current.getValueAsDouble() > 30.0 && getLinearDistanceInches() < 2.0;
    }

    @Override
    public void runCharacterization(double voltage) {
        double voltageRequest = MathUtil.clamp(voltage * 12, -12.0, 12.0);
        climbMotor.setVoltage(voltageRequest);
    }

    @Override
    public void setFF(double kS, double kV, double kA) {
        config.Slot0.kS = kS;
        config.Slot0.kV = kV;
        config.Slot0.kA = kA;

        this.climbMotor.getConfigurator().apply(config);
    }

    @Override
    public void setPID(double kP, double kI, double kD) {
        config.Slot0.kP = kP;
        config.Slot0.kI = kI;
        config.Slot0.kD = kD;

        this.climbMotor.getConfigurator().apply(config);
    }

    @Override
    public void setPosition(double inches, boolean isHooked) {
        double targetRotations = inches / INCHES_PER_ROTATION;
        if (!isHooked) {
            climbMotor.setControl(motionMagicRequest.withPosition(targetRotations).withSlot(0));
        } else {
            climbMotor.setControl(motionMagicRequest.withPosition(targetRotations).withSlot(1));
        }
    }

    @Override
    public void stop() {
        climbMotor.setControl(new VoltageOut(0));
    }

    @Override
    public void zeroPosition() {
        climbMotor.setControl(new NeutralOut());
        climbMotor.setPosition(0);
    }
}