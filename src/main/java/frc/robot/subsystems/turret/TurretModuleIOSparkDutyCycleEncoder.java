package frc.robot.subsystems.turret;

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
import frc.robot.Constants.turretConstants;

/**
 * 
 */
public class TurretModuleIOSparkDutyCycleEncoder implements TurretModuleIO {
    // Constants
    protected final double GEAR_RATIO = 28.6667 * 12;

    // Hardware
    protected final SparkMax turretMotor;
    private final DigitalInput hallEffect;
    private final DutyCycleEncoder turretEncoder;

    // PID
    private final ProfiledPIDController pidController;
    private final SimpleMotorFeedforward feedforward;
    private TrapezoidProfile.Constraints feedForwardConstraints;

    // Connection debouncers
    private final Debouncer connectedDebouncer;

    /**
     * 
     */
    public TurretModuleIOSparkDutyCycleEncoder() {
        this.turretMotor = new SparkMax(turretConstants.turretMotorChannel, SparkLowLevel.MotorType.kBrushless);
        this.hallEffect = new DigitalInput(turretConstants.hallEffectChannel);
        this.turretEncoder = new DutyCycleEncoder(turretConstants.turretEncoderChannel, 1, 0);
        this.turretEncoder.setDutyCycleRange(0.1, 0.9);

        this.feedForwardConstraints = new TrapezoidProfile.Constraints(turretConstants.turretMaxSpeed,
                turretConstants.turretMaxAccel);
        this.pidController = new ProfiledPIDController(turretConstants.kP, turretConstants.kI, turretConstants.kD,
                feedForwardConstraints);
        this.pidController.setConstraints(feedForwardConstraints);
        this.pidController.setTolerance(turretConstants.turretTolerance);
        this.feedforward = new SimpleMotorFeedforward(turretConstants.kS, turretConstants.kV, turretConstants.kA);

        this.connectedDebouncer = new Debouncer(0.5);
    }

    @Override
    public void updateInputs(TurretModuleIOInputs inputs) {
        if (DriverStation.isDisabled()) {
            stop();
        }

        double pValue = pidController.calculate(getCurrentAngle());
        double fValue = feedforward.calculate(pidController.getSetpoint().velocity);
        turretMotor.setVoltage(pValue + fValue);

        inputs.data = new TurretModuleIOData(
                connectedDebouncer.calculate(true), // TODO: add spark utility to test for connection
                getPosition(),
                getVelocity(),
                turretMotor.getAppliedOutput(),
                0.0,
                turretMotor.getOutputCurrent(),
                turretMotor.getMotorTemperature());
    }

    @Override
    public boolean atGoal() {
        return pidController.atGoal();
    }

    @Override
    public double getCurrentAngle() {
        // double motorRotations = turretModuleIO.getPosition();
        double turretRotations = turretMotor.getEncoder().getPosition() / GEAR_RATIO;
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
    public boolean getHallEffect() {
        return hallEffect.get();
    }

    @Override
    public double getPosition() {
        return turretMotor.getEncoder().getPosition();
    }

    @Override
    public double getVelocity() {
        // double currentVelocity = turretMotor.getEncoder().getVelocity();
        // double DegPerSec = currentVelocity * 360/60;
        // return DegPerSec;

        // turretMotor.getEncoder().getVelocity() returns motor RPM (rotations per
        // minute)

        // double motorRPM = turretModuleIO.getVelocity();
        double motorRPM = turretMotor.getEncoder().getVelocity();

        // Convert motor RPM -> turret rotations per second by dividing by gear ratio
        // and 60
        double turretRPS = (motorRPM / GEAR_RATIO) / 60.0;
        // Convert rotations per second -> degrees per second
        double degPerSec = turretRPS * 360.0;
        return degPerSec;
    }

    @Override
    public void setAngle(double degrees) {
        pidController.setGoal(MathUtil.clamp(degrees, -turretConstants.ANGLE_LIMIT, turretConstants.ANGLE_LIMIT));
    }

    @Override
    public void setConstraints(double maxVelocity, double maxAcceleration, double tolerance) {
        feedForwardConstraints = new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration);
        pidController.setConstraints(feedForwardConstraints);
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
        turretMotor.getEncoder().setPosition(position);
    }

    @Override
    public void setVoltage() {
        double pValue = pidController.calculate(getCurrentAngle());
        double fValue = feedforward.calculate(pidController.getSetpoint().velocity);
        double voltageRequest = MathUtil.clamp(pValue + fValue, -12.0, 12.0);

        turretMotor.setVoltage(voltageRequest);
    }

    @Override
    public void stop() {
        turretMotor.set(0.0);
    }
}
