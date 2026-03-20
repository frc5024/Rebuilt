package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants;
import frc.robot.Constants.TurretConstants;

/**
 * 
 */
public class TurretModuleIOSparkMaxExternalPID implements TurretModuleIO {
    // Constants
    protected final double GEAR_RATIO = 28.6667; // 129 ring gear, 18 pinion, 4:1 internal = (129.0 / 18.0) * 4

    // Hardware
    protected final SparkMax turretMotor;
    protected final DigitalInput hallEffectSensor;
    protected final DutyCycleEncoder absoluteEncoder;
    protected final RelativeEncoder internalEncoder;

    // PID
    private final ProfiledPIDController pidController;
    private final TrapezoidProfile.Constraints feedForwardConstraints;
    private final SimpleMotorFeedforward feedforward;

    // Connection debouncers
    private final Debouncer connectedDebouncer;

    /**
     * 
     */
    public TurretModuleIOSparkMaxExternalPID() {
        this.turretMotor = new SparkMax(Constants.TurretConstants.MOTOR_ID,
                SparkLowLevel.MotorType.kBrushless);
        this.hallEffectSensor = new DigitalInput(TurretConstants.hallEffectChannel);
        this.absoluteEncoder = new DutyCycleEncoder(TurretConstants.turretEncoderChannel, 1, 0);
        this.absoluteEncoder.setDutyCycleRange(0.1, 0.9);
        this.internalEncoder = this.turretMotor.getEncoder();

        this.feedForwardConstraints = new TrapezoidProfile.Constraints(TurretConstants.turretMaxSpeed,
                TurretConstants.turretMaxAccel);
        this.pidController = new ProfiledPIDController(Constants.TurretConstants.kP,
                Constants.TurretConstants.kI, Constants.TurretConstants.kD, feedForwardConstraints);
        this.pidController.setConstraints(feedForwardConstraints);
        this.pidController.setTolerance(Constants.TurretConstants.turretTolerance);

        this.feedforward = new SimpleMotorFeedforward(TurretConstants.kS,
                TurretConstants.kV, TurretConstants.kA);

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
        // double motorRotations = turretModuleIO.getPosition();
        double turretRotations = internalEncoder.getPosition() / GEAR_RATIO;
        // returns degrees normalized to [-180, 180)
        double angle = turretRotations * 360.0;
        while (angle >= 180.0) {
            angle -= 360.0;
        }
        while (angle < -180.0) {
            angle += 360.0;
        }
        return angle;
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
        // double currentVelocity = internalEncoder.getVelocity();
        // double DegPerSec = currentVelocity * 360/60;
        // return DegPerSec;

        // internalEncoder.getVelocity() returns motor RPM (rotations per
        // minute)

        // double motorRPM = turretModuleIO.getVelocity();
        double motorRPM = internalEncoder.getVelocity();

        // Convert motor RPM -> turret rotations per second by dividing by gear ratio
        // and 60
        double turretRPS = (motorRPM / GEAR_RATIO) / 60.0;
        // Convert rotations per second -> degrees per second
        double degPerSec = turretRPS * 360.0;
        return degPerSec;
    }

    @Override
    public void setAngle(double degrees) {
        pidController.setGoal(MathUtil.clamp(degrees, -TurretConstants.ANGLE_LIMIT, TurretConstants.ANGLE_LIMIT));
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

        Logger.recordOutput("Turret/pValue", pValue);
        Logger.recordOutput("Turret/fValue", fValue);
        Logger.recordOutput("Turret/Setpoint", pidController.getSetpoint());
        turretMotor.setVoltage(voltageRequest);
    }
}
