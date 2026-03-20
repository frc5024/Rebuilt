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
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants.TurretConstants;

/**
 * 
 */
public class TurretModuleIOSparkMaxDutyCycleEncoder implements TurretModuleIO {
    // Constants
    protected final double GEAR_RATIO = 28.6667; // 129 ring gear, 18 pinion, 4:1 internal = (129.0 / 18.0) * 4

    // Hardware
    protected final SparkMax turretMotor;
    protected final DigitalInput hallEffectSensor;
    protected final DutyCycleEncoder absoluteEncoder;
    protected final RelativeEncoder internalEncoder;

    // PID
    private final ProfiledPIDController pidController;
    private TrapezoidProfile.Constraints constraints;

    private final SimpleMotorFeedforward feedforward;

    // Connection debouncers
    private final Debouncer connectedDebouncer;

    /**
     * 
     */
    public TurretModuleIOSparkMaxDutyCycleEncoder() {
        this.turretMotor = new SparkMax(TurretConstants.MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
        this.hallEffectSensor = new DigitalInput(TurretConstants.hallEffectChannel);
        this.absoluteEncoder = new DutyCycleEncoder(TurretConstants.turretEncoderChannel, 1, 0);
        this.absoluteEncoder.setDutyCycleRange(0.1, 0.9);
        this.internalEncoder = this.turretMotor.getEncoder();

        // get and set constraints and PID constants
        double[] kPIDs = TurretConstants.getPIDs();
        this.constraints = new TrapezoidProfile.Constraints(TurretConstants.turretMaxSpeed,
                TurretConstants.turretMaxAccel);

        this.pidController = new ProfiledPIDController(kPIDs[0], kPIDs[1], kPIDs[2], constraints);
        this.pidController.setConstraints(constraints);
        this.pidController.setTolerance(TurretConstants.turretTolerance);

        // get and set feedforward SVA constants
        double[] kSVAs = TurretConstants.getSVAs();
        this.feedforward = new SimpleMotorFeedforward(kSVAs[0], kSVAs[1], kSVAs[2]);

        // set position factor so we can turn turret to specific angle
        SparkMaxConfig config = new SparkMaxConfig();

        // Set idle mode and current limit
        config.idleMode(IdleMode.kBrake)
                .smartCurrentLimit(20);

        // limit turn angle due to wiring
        config.softLimit
                .forwardSoftLimit(TurretConstants.ANGLE_LIMIT)
                .forwardSoftLimitEnabled(true)
                .reverseSoftLimit(-TurretConstants.ANGLE_LIMIT)
                .reverseSoftLimitEnabled(true);

        double positionFactor = 360.0 / GEAR_RATIO;
        config.encoder
                .positionConversionFactor(positionFactor)
                .velocityConversionFactor(positionFactor / 60.0);

        this.turretMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        this.connectedDebouncer = new Debouncer(0.5);
    }

    @Override
    public void updateInputs(TurretModuleIOInputs inputs) {
        if (DriverStation.isDisabled()) {
            stop();
        }

        inputs.data = new TurretModuleIOData(
                connectedDebouncer.calculate(true), // TODO: add spark utility to test for connection
                getPosition(),
                getVelocity(),
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
    public double getCurrentAngle() {
        return internalEncoder.getPosition();
    }

    @Override
    public double getCurrentDrawAmps() {
        return turretMotor.getAppliedOutput();
    }

    @Override
    public double getGoalPosition() {
        return pidController.getGoal().position;
    }

    @Override
    public boolean getHallEffectValue() {
        return hallEffectSensor.get();
    }

    @Override
    public double getPosition() {
        return internalEncoder.getPosition();
    }

    @Override
    public double getVelocity() {
        return internalEncoder.getVelocity();
    }

    @Override
    public void setAngle(double degrees) {
        pidController.setGoal(MathUtil.clamp(degrees, -TurretConstants.ANGLE_LIMIT, TurretConstants.ANGLE_LIMIT));
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
        internalEncoder.setPosition(position);
    }

    @Override
    public void setVoltage() {
        double pValue = pidController.calculate(getCurrentAngle());
        double fValue = feedforward.calculate(pidController.getSetpoint().velocity);
        double voltageRequest = MathUtil.clamp(pValue + fValue, -12.0, 12.0);

        turretMotor.setVoltage(voltageRequest);

        Logger.recordOutput("Turret/pValue", pValue);
        Logger.recordOutput("Turret/fValue", fValue);
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
