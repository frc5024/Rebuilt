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
public class ShooterModuleIOSparkFlexClosedLoopController implements ShooterModuleIO {
    // Constants
    private final int LEAD_MOTOR_ID = 51;
    private final int FOLLOWER_MOTOR_ID = 52;

    protected final double GEAR_RATIO = 1.0;
    private final double RPM_TOLERANCE = 200.0;

    // Hardware
    protected final SparkFlex leadMotor;
    private final SparkFlex followerMotor;
    protected final RelativeEncoder encoder;
    private final SparkClosedLoopController pidController;

    private SparkFlexConfig leadConfig;
    private SparkFlexConfig followerConfig;

    // Connection debouncers
    private final Debouncer leadConnectedDebouncer;
    private final Debouncer followerConnectedDebouncer;

    /**
     * 
     */
    public ShooterModuleIOSparkFlexClosedLoopController() {
        this.leadMotor = new SparkFlex(LEAD_MOTOR_ID, MotorType.kBrushless);
        this.followerMotor = new SparkFlex(FOLLOWER_MOTOR_ID, MotorType.kBrushless);
        this.encoder = this.leadMotor.getEncoder();
        this.pidController = this.leadMotor.getClosedLoopController();

        this.leadConfig = new SparkFlexConfig();
        this.followerConfig = new SparkFlexConfig();

        leadConfig.idleMode(IdleMode.kCoast)
                .inverted(true)
                .smartCurrentLimit(60);

        // set PIDs and kSVAs
        double[] kPIDs = ShooterConstants.getPIDs();
        double[] kSVAs = ShooterConstants.getSVAs();
        leadConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(kPIDs[0], kPIDs[1], kPIDs[2]).feedForward
                .kS(kSVAs[0])
                .kV(kSVAs[1])
                .kA(kSVAs[2]);

        followerConfig
                .idleMode(IdleMode.kCoast)
                .inverted(false)
                .follow(LEAD_MOTOR_ID)
                .smartCurrentLimit(60);

        this.leadMotor.configure(leadConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        this.followerMotor.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        this.leadConnectedDebouncer = new Debouncer(0.5);
        this.followerConnectedDebouncer = new Debouncer(0.5);
    }

    @Override
    public void updateInputs(ShooterModuleIOInputs inputs) {
        if (DriverStation.isDisabled()) {
            stop();
        }

        inputs.data = new ShooterModuleIOData(
                leadConnectedDebouncer.calculate(true),
                encoder.getPosition(),
                encoder.getVelocity(),
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
    public double getPosition() {
        return encoder.getPosition();
    }

    @Override
    public double getVelocity() {
        return encoder.getVelocity();
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
