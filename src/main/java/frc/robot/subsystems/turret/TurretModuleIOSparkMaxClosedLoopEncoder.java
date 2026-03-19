package frc.robot.subsystems.turret;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.TurretConstants;

public class TurretModuleIOSparkMaxClosedLoopEncoder implements TurretModuleIO {
    // Constants
    protected final double GEAR_RATIO = 28.6667; // 129 ring gear, 18 pinion, 4:1 internal = (129.0 / 18.0) * 4

    // Hardware
    protected final SparkMax turretMotor;
    protected final DigitalInput hallEffectSensor;
    protected final SparkClosedLoopController pidController;
    protected final RelativeEncoder internalEncoder;

    private final SparkMaxConfig config;

    // Connection debouncers
    private final Debouncer connectedDebouncer;

    /**
     * 
     */
    public TurretModuleIOSparkMaxClosedLoopEncoder() {
        this.turretMotor = new SparkMax(TurretConstants.MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
        this.hallEffectSensor = new DigitalInput(TurretConstants.hallEffectChannel);
        this.pidController = this.turretMotor.getClosedLoopController();
        this.internalEncoder = this.turretMotor.getEncoder();
        this.config = new SparkMaxConfig();

        this.connectedDebouncer = new Debouncer(0.5);

        // set position factor so we can turn turret to specific angle
        double positionFactor = 360.0 / GEAR_RATIO;
        config.encoder
                .positionConversionFactor(positionFactor)
                .velocityConversionFactor(positionFactor / 60.0);

        // limit turn angle sue to wiring
        config.softLimit
                .forwardSoftLimit(TurretConstants.ANGLE_LIMIT)
                .forwardSoftLimitEnabled(true)
                .reverseSoftLimit(-TurretConstants.ANGLE_LIMIT)
                .reverseSoftLimitEnabled(true);

        // Set current limit
        config.smartCurrentLimit(20);

        // get and set PID constants
        double[] kPIDs = TurretConstants.getPIDs();
        double[] kSVAs = TurretConstants.getSVAs();
        config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .p(kPIDs[0])
                .i(kPIDs[1])
                .d(kPIDs[2]).feedForward
                .kS(kSVAs[0])
                .kV(kSVAs[1])
                .kA(kSVAs[2]);

        this.turretMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(TurretModuleIOInputs inputs) {
        if (DriverStation.isDisabled()) {
            stop();
        }

        inputs.data = new TurretModuleIOData(
                connectedDebouncer.calculate(true),
                getPosition(),
                getVelocity(),
                turretMotor.getAppliedOutput(),
                0.0,
                turretMotor.getOutputCurrent(),
                turretMotor.getMotorTemperature(),
                false);
    }

    @Override
    public boolean atGoal() {
        return pidController.isAtSetpoint();
    }

    @Override
    public double getCurrentDrawAmps() {
        return turretMotor.getOutputCurrent();
    }

    @Override
    public double getCurrentAngle() {
        return internalEncoder.getPosition();
    }

    @Override
    public double getGoalPosition() {
        return pidController.getSetpoint();
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
        double target = Math.max(-TurretConstants.ANGLE_LIMIT, Math.min(TurretConstants.ANGLE_LIMIT, degrees));
        pidController.setSetpoint(target, ControlType.kPosition);
    }

    @Override
    public void setPID(double kP, double kI, double kD) {
        config.closedLoop.p(kP).i(kI).d(kD);

        turretMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void setFF(double kS, double kV, double kA) {
        config.closedLoop.feedForward.kS(kS).kV(kV).kA(kA);

        turretMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void stop() {
        turretMotor.stopMotor();
    }
}
