package frc.robot.subsystems.intake;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.IntakeConstants.RollerConstants;

/**
 * 
 */
public class RollerModuleIOSparkFlexClosedLoopController implements RollerModuleIO {
    // Constants
    protected final int MOTOR_ID = 60;
    protected final double GEAR_RATIO = 1.33;

    // Hardware
    protected final SparkFlex rollerMotor;
    protected final RelativeEncoder rollerEncoder;
    private final SparkClosedLoopController pidController;

    private SparkFlexConfig config;

    // Connection debouncers
    private final Debouncer connectedDebouncer;

    /**
     * 
     */
    public RollerModuleIOSparkFlexClosedLoopController() {
        this.rollerMotor = new SparkFlex(MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
        this.rollerEncoder = this.rollerMotor.getEncoder();
        this.pidController = this.rollerMotor.getClosedLoopController();

        this.config = new SparkFlexConfig();
        config.idleMode(SparkFlexConfig.IdleMode.kCoast);

        // set velocity factor
        double velocityConversionFactor = 1 / GEAR_RATIO;
        config.encoder
                .velocityConversionFactor(velocityConversionFactor);

        // set PIDs and kSVAs
        double[] kPIDs = RollerConstants.getPIDs();
        double[] kSVAs = RollerConstants.getSVAs();
        config.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(kPIDs[0], kPIDs[1], kPIDs[2]).feedForward
                .kS(kSVAs[0])
                .kV(kSVAs[1])
                .kA(kSVAs[2]);

        // Set current limit
        config.smartCurrentLimit(20);

        this.rollerMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        this.connectedDebouncer = new Debouncer(0.5);
    }

    @Override
    public void updateInputs(RollerModuleIOInputs inputs) {
        if (DriverStation.isDisabled()) {
            stop();
        }

        inputs.data = new RollerModuleIOData(
                connectedDebouncer.calculate(true),
                rollerEncoder.getPosition(),
                rollerEncoder.getVelocity(),
                rollerMotor.getAppliedOutput(),
                0.0,
                rollerMotor.getOutputCurrent(),
                rollerMotor.getMotorTemperature());
    }

    @Override
    public double getCurrentDrawAmps() {
        return rollerMotor.getOutputCurrent();
    }

    @Override
    public double getFFCharacterizationVelocity() {
        return Units.radiansToRotations(rollerMotor.getAppliedOutput());
    }

    @Override
    public double getGoalVelocity() {
        return pidController.getSetpoint();
    }

    @Override
    public double getPosition() {
        return rollerEncoder.getPosition();
    }

    @Override
    public double getVelocity() {
        return rollerEncoder.getVelocity();
    }

    @Override
    public void intake() {
        pidController.setSetpoint(RollerConstants.INTAKE_RPM, ControlType.kVelocity);
    }

    @Override
    public void outtake() {
        pidController.setSetpoint(RollerConstants.OUTTAKE_RPM, ControlType.kVelocity);
    }

    @Override
    public void runCharacterization(double voltage) {
        double voltageRequest = MathUtil.clamp(voltage * 12, -12.0, 12.0);
        rollerMotor.set(voltageRequest);
    }

    @Override
    public void setPID(double kP, double kI, double kD) {
        config.closedLoop.p(kP).i(kI).d(kD);

        rollerMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void setFF(double kS, double kV, double kA) {
        config.closedLoop.feedForward.kS(kS).kV(kV).kA(kA);

        rollerMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void stop() {
        rollerMotor.stopMotor();
    }
}
