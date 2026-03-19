package frc.robot.subsystems.shooter;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.ShooterConstants;

/**
 * 
 */
public class ShooterModuleIOSparkFlexClosedLoopEncoder implements ShooterModuleIO {
    // Constants
    private final int LEAD_MOTOR_ID = 51;
    private final int FOLLOWER_MOTOR_ID = 52;
    protected final double GEAR_RATIO = 1.0;
    private final double RPM_TOLERANCE = 200.0;

    // Hardware
    protected final SparkFlex leadMotor;
    private final SparkFlex followerMotor;
    private final SparkClosedLoopController pidController;
    protected final RelativeEncoder internalEncoder;

    private SparkFlexConfig leadConfig;
    private SparkFlexConfig followerConfig;

    // Connection debouncers
    private final Debouncer leadConnectedDebouncer;
    private final Debouncer followerConnectedDebouncer;

    /**
     * 
     */
    public ShooterModuleIOSparkFlexClosedLoopEncoder() {
        this.leadMotor = new SparkFlex(LEAD_MOTOR_ID, MotorType.kBrushless);
        this.followerMotor = new SparkFlex(FOLLOWER_MOTOR_ID, MotorType.kBrushless);
        this.pidController = this.leadMotor.getClosedLoopController();
        this.internalEncoder = this.leadMotor.getEncoder();

        this.leadConnectedDebouncer = new Debouncer(0.5);
        this.followerConnectedDebouncer = new Debouncer(0.5);

        this.leadConfig = new SparkFlexConfig();
        this.followerConfig = new SparkFlexConfig();

        leadConfig.smartCurrentLimit(60)
                .idleMode(IdleMode.kCoast)
                .inverted(true);

        leadConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .p(ShooterConstants.kP)
                .i(ShooterConstants.kI)
                .d(ShooterConstants.kD).feedForward
                .kS(ShooterConstants.kS)
                .kV(ShooterConstants.kV)
                .kA(ShooterConstants.kA);

        followerConfig.follow(LEAD_MOTOR_ID)
                .smartCurrentLimit(60)
                .idleMode(IdleMode.kCoast);

        this.leadMotor.configure(leadConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        this.followerMotor.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(ShooterModuleIOInputs inputs) {
        if (DriverStation.isDisabled()) {
            stop();
        }

        inputs.data = new ShooterModuleIOData(
                leadConnectedDebouncer.calculate(true),
                internalEncoder.getPosition(),
                internalEncoder.getVelocity(),
                leadMotor.getAppliedOutput(),
                leadMotor.getBusVoltage(),
                leadMotor.getOutputCurrent(),
                leadMotor.getMotorTemperature(),
                followerConnectedDebouncer.calculate(true),
                followerMotor.getEncoder().getPosition(),
                followerMotor.getEncoder().getVelocity(),
                followerMotor.getAppliedOutput(),
                followerMotor.getBusVoltage(),
                followerMotor.getOutputCurrent(),
                followerMotor.getMotorTemperature());
    }

    @Override
    public double getCurrentDrawAmps() {
        return leadMotor.getOutputCurrent() + followerMotor.getOutputCurrent();
    }

    @Override
    public double getGoalVelocity() {
        return pidController.getSetpoint();
    }

    @Override
    public double getSetpoint() {
        return pidController.getSetpoint();
    }

    @Override
    public double getVelocity() {
        return internalEncoder.getVelocity();
    }

    @Override
    public boolean isAtSetpoint() {
        return Math.abs(getVelocity() - pidController.getSetpoint()) < RPM_TOLERANCE;
    }

    @Override
    public void setPID(double kP, double kI, double kD) {
        leadConfig.closedLoop.p(kP).i(kI).d(kD);

        leadMotor.configure(leadConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void setFF(double kS, double kV, double kA) {
        leadConfig.closedLoop.feedForward.kS(kS).kV(kV).kA(kA);

        leadMotor.configure(leadConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void setVelocity(double rpm) {
        pidController.setSetpoint(rpm, ControlType.kVelocity);
    }

    @Override
    public void stop() {
        leadMotor.stopMotor();
    }
}
