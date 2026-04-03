package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.TurretConstants;

/**
 * 
 */
public class TurretModuleIOSparkMaxRelativeEncoder implements TurretModuleIO {
    // Constants
    private final int MOTOR_ID = 3;
    private final int HALLEFFECT_CHANNEL = 3;
    private final int ENCODER_CHANNEL = 0;
    protected final double GEAR_RATIO = 28.6667; // 129 ring gear, 18 pinion, 4:1 internal = (129.0 / 18.0) * 4

    // Hardware
    protected final SparkMax turretMotor;
    protected final RelativeEncoder encoder;
    protected final DigitalInput hallEffectSensor;

    // PID
    private final SimpleMotorFeedforward feedforward;
    private final ProfiledPIDController pidController;
    private TrapezoidProfile.Constraints constraints;

    // Connection debouncers
    private final Debouncer connectedDebouncer;

    /**
     * 
     */
    public TurretModuleIOSparkMaxRelativeEncoder() {
        this.turretMotor = new SparkMax(MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
        this.hallEffectSensor = new DigitalInput(HALLEFFECT_CHANNEL);
        this.encoder = this.turretMotor.getEncoder();

        SparkMaxConfig config = new SparkMaxConfig();
        config
                .idleMode(IdleMode.kBrake)
                .inverted(false)
                .smartCurrentLimit(40);

        // limit turn angle due to wiring
        config.softLimit
                .forwardSoftLimit(TurretConstants.ANGLE_LIMIT)
                .forwardSoftLimitEnabled(true)
                .reverseSoftLimit(-TurretConstants.ANGLE_LIMIT)
                .reverseSoftLimitEnabled(true);

        // set position factor so we can turn turret to specific angle
        double positionConversionFactor = 360.0 / GEAR_RATIO;
        config.encoder
                .positionConversionFactor(positionConversionFactor)
                .velocityConversionFactor(positionConversionFactor / 60.0);

        this.turretMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // get and set feedforward SVA constants
        double[] kSVAs = TurretConstants.getSVAs();
        this.feedforward = new SimpleMotorFeedforward(kSVAs[0], kSVAs[1], kSVAs[2]);

        // get and set constraints and PID constants
        double[] kPIDs = TurretConstants.getPIDs();
        this.constraints = new TrapezoidProfile.Constraints(TurretConstants.MAX_SPEED,
                TurretConstants.MAX_ACCEL);
        this.pidController = new ProfiledPIDController(kPIDs[0], kPIDs[1], kPIDs[2], constraints);
        this.pidController.setTolerance(TurretConstants.TOLERANCE);

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
                turretMotor.getMotorTemperature(),
                !hallEffectSensor.get());
    }

    @Override
    public boolean atGoal() {
        return pidController.atGoal();
    }

    @Override
    public double getCurrentDrawAmps() {
        return turretMotor.getAppliedOutput();
    }

    @Override
    public double getFFCharacterizationVelocity() {
        return encoder.getVelocity();
    }

    @Override
    public boolean getHallEffectValue() {
        return hallEffectSensor.get();
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
    public void runCharacterization(double voltage) {
        double voltageRequest = MathUtil.clamp(voltage * 12, -12.0, 12.0);
        turretMotor.setVoltage(voltageRequest);
    }

    @Override
    public void setAngle(double targetAngle) {
        // calculate for targetAngle
        double currentAngle = encoder.getPosition();

        double pidVoltage = pidController.calculate(currentAngle, targetAngle);
        double ffVoltage = feedforward.calculate(pidController.getSetpoint().velocity);

        double voltageRequest = MathUtil.clamp(ffVoltage + pidVoltage, -12.0, 12.0);

        if (targetAngle == currentAngle) {
            turretMotor.setVoltage(0.0);
            pidController.reset(currentAngle);
        } else {
            turretMotor.setVoltage(voltageRequest);
        }

        Logger.recordOutput("Subsystems/Turret/ffVoltage", ffVoltage);
        Logger.recordOutput("Subsystems/Turret/pidVoltage", pidVoltage);
    }

    @Override
    public void setConstraints(double maxVelocity, double maxAcceleration, double tolerance) {
        constraints = new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration);
        pidController.setConstraints(constraints);
        pidController.setTolerance(tolerance);
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
    public void setPosition(double position) {
        encoder.setPosition(position);
    }

    @Override
    public void set(double speed) {
        turretMotor.set(speed);
    }

    @Override
    public void stop() {
        turretMotor.stopMotor();
    }
}
