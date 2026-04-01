package frc.robot.subsystems.intake;

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
import frc.robot.Constants.IntakeConstants.ArmConstants;

/**
 * 
 */
public class ArmModuleIOSparkMaxRelativeEncoder implements ArmModuleIO {
    // Constants
    protected final int MOTOR_ID = 5;
    protected final int RETRACTED_ID = 7;
    protected final int EXTENDED_ID = 8;

    protected final double GEAR_RATIO = 13.57;

    // Hardware
    protected final SparkMax armMotor;
    protected final RelativeEncoder encoder;
    protected final DigitalInput retractedLimit;
    protected final DigitalInput extendedLimit;

    // PID
    private final SimpleMotorFeedforward feedforward;
    private final ProfiledPIDController pidController;
    private TrapezoidProfile.Constraints constraints;

    // Connection debouncers
    private final Debouncer connectedDebouncer;

    /**
     * 
     */
    public ArmModuleIOSparkMaxRelativeEncoder() {
        this.armMotor = new SparkMax(MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
        this.retractedLimit = new DigitalInput(RETRACTED_ID);
        this.extendedLimit = new DigitalInput(EXTENDED_ID);
        this.encoder = this.armMotor.getEncoder();

        SparkMaxConfig config = new SparkMaxConfig();
        config
                .idleMode(IdleMode.kBrake)
                .inverted(true)
                .smartCurrentLimit(60);

        // set the arm limits by degree
        config.softLimit
                .forwardSoftLimit(ArmConstants.EXTENDED_ANGLE)
                .forwardSoftLimitEnabled(true)
                .reverseSoftLimit(ArmConstants.RETRACTED_ANGLE)
                .reverseSoftLimitEnabled(true);

        // set position factor so we can set arm to specific angle
        double positionConversionFactor = 360.0 / GEAR_RATIO;
        config.encoder
                .positionConversionFactor(positionConversionFactor)
                .velocityConversionFactor(positionConversionFactor / 60.0);

        this.armMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // get and set feedforward sva constants
        double[] kSVAs = ArmConstants.getSVAs();
        this.feedforward = new SimpleMotorFeedforward(kSVAs[0], kSVAs[1], kSVAs[2]);

        // get and set contraints and pid constants
        double[] kPIDs = ArmConstants.getPIDs();
        this.constraints = new TrapezoidProfile.Constraints(ArmConstants.MAX_SPEED, ArmConstants.MAX_ACCEL);
        this.pidController = new ProfiledPIDController(kPIDs[0], kPIDs[1], kPIDs[2], constraints);
        this.pidController.setTolerance(ArmConstants.TOLERANCE);

        this.connectedDebouncer = new Debouncer(0.5);
    }

    @Override
    public void updateInputs(ArmModuleIOInputs inputs) {
        if (DriverStation.isDisabled()) {
            stop();
        }

        inputs.data = new ArmModuleIOData(
                connectedDebouncer.calculate(true),
                getPosition(),
                getVelocity(),
                armMotor.getAppliedOutput(),
                0.0,
                armMotor.getOutputCurrent(),
                armMotor.getMotorTemperature());
    }

    @Override
    public double getCurrentDrawAmps() {
        return armMotor.getOutputCurrent();
    }

    @Override
    public double getFFCharacterizationVelocity() {
        return encoder.getVelocity();
    }

    @Override
    public double getGoalPosition() {
        return pidController.getGoal().position;
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
    public boolean isExtended() {
        return !extendedLimit.get();
    }

    @Override
    public boolean isRetracted() {
        return !retractedLimit.get();
    }

    @Override
    public void runCharacterization(double voltage) {
        double voltageRequest = MathUtil.clamp(voltage * 12, -12.0, 12.0);
        armMotor.setVoltage(voltageRequest);
    }

    @Override
    public void setAngle(double targetAngle) {
        // calculate for targetAngle
        double currentAngle = encoder.getPosition();

        double pidVoltage = pidController.calculate(currentAngle, targetAngle);
        double ffVoltage = feedforward.calculate(pidController.getSetpoint().velocity);

        double voltageRequest = MathUtil.clamp(ffVoltage + pidVoltage, -12.0, 12.0);

        if (targetAngle == currentAngle) {
            armMotor.setVoltage(0.0);
            pidController.reset(currentAngle);
        } else {
            armMotor.setVoltage(voltageRequest);
        }

        Logger.recordOutput("Intake/Arm/ffVoltage", ffVoltage);
        Logger.recordOutput("Intake/Arm/pidVoltage", pidVoltage);
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
    public void stop() {
        armMotor.stopMotor();
    }
}
