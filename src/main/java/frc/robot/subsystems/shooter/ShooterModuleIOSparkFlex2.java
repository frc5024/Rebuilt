package frc.robot.subsystems.shooter;

import com.revrobotics.PersistMode;
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
import frc.robot.Constants.shooterConstants;

/**
 * 
 */
public class ShooterModuleIOSparkFlex2 implements ShooterModuleIO {
    // Constants
    private final int LEAD_MOTOR_ID = 51;
    private final int FOLLOWER_MOTOR_ID = 52;
    public final double GEAR_RATIO = 1.0;
    private final double VELOCITY_TOLERANCE_RPM = 50.0; // Acceptable error range

    // Hardware
    protected final SparkFlex leadMotor;
    private final SparkFlex followerMotor;
    private final SparkClosedLoopController pidController;
    private final SparkFlexConfig leadConfig;
    private final SparkFlexConfig followerConfig;

    // Connection debouncers
    private final Debouncer leadConnectedDebouncer;
    private final Debouncer followerConnectedDebouncer;

    /**
     * 
     */
    public ShooterModuleIOSparkFlex2() {
        this.leadMotor = new SparkFlex(LEAD_MOTOR_ID, MotorType.kBrushless);
        this.followerMotor = new SparkFlex(FOLLOWER_MOTOR_ID, MotorType.kBrushless);
        this.pidController = this.leadMotor.getClosedLoopController();

        this.leadConnectedDebouncer = new Debouncer(0.5);
        this.followerConnectedDebouncer = new Debouncer(0.5);

        this.leadConfig = new SparkFlexConfig();
        this.followerConfig = new SparkFlexConfig();

        leadConfig.smartCurrentLimit(60)
                .idleMode(IdleMode.kCoast)
                .inverted(true);

        leadConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .p(shooterConstants.kP)
                .i(shooterConstants.kI)
                .d(shooterConstants.kD).feedForward
                .kS(shooterConstants.kS)
                .kV(shooterConstants.kV)
                .kA(shooterConstants.kA);

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
                leadConnectedDebouncer.calculate(leadMotor.getFaults().other),
                leadMotor.getEncoder().getPosition(),
                leadMotor.getEncoder().getVelocity(),
                leadMotor.getAppliedOutput(),
                leadMotor.getBusVoltage(),
                leadMotor.getOutputCurrent(),
                leadMotor.getMotorTemperature(),
                followerConnectedDebouncer.calculate(followerMotor.getFaults().other),
                followerMotor.getEncoder().getPosition(),
                followerMotor.getEncoder().getVelocity(),
                followerMotor.getAppliedOutput(),
                followerMotor.getBusVoltage(),
                followerMotor.getOutputCurrent(),
                followerMotor.getMotorTemperature());
    }

    @Override
    public double getSetpoint() {
        return pidController.getSetpoint();
    }

    @Override
    public double getVelocity() {
        return leadMotor.getEncoder().getVelocity();
    }

    @Override
    public boolean isAtSetpoint() {
        return Math.abs(getVelocity() - pidController.getSetpoint()) < VELOCITY_TOLERANCE_RPM;
    }

    @Override
    public void setPID(double kP, double kI, double kD) {
        leadConfig.closedLoop.p(kP).i(kI).d(kD);

        this.leadMotor.configure(leadConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void setFF(double kS, double kV, double kA) {
        leadConfig.closedLoop.feedForward.kS(kS).kV(kV).kA(kA);

        this.leadMotor.configure(leadConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void setVelocity(double rpm) {
        pidController.setSetpoint(rpm, ControlType.kVelocity);
    }
}
