package frc.robot.subsystems.turret;

import com.revrobotics.PersistMode;
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
import frc.robot.Constants.turretConstants;

public class TurretModuleIOSparkMaxEncoderPID implements TurretModuleIO {
    // Constants
    protected final double GEAR_RATIO = 28.6667; // 129 ring gear, 18 pinion, 4:1 internal = (129.0 / 18.0) * 4

    // Hardware
    protected final SparkMax turretMotor;
    protected final DigitalInput hallEffectSensor;
    protected final SparkClosedLoopController pidController;

    private final SparkMaxConfig config;

    // Connection debouncers
    private final Debouncer connectedDebouncer;

    /**
     * 
     */
    public TurretModuleIOSparkMaxEncoderPID() {
        this.turretMotor = new SparkMax(turretConstants.turretMotorChannel, SparkLowLevel.MotorType.kBrushless);
        this.hallEffectSensor = new DigitalInput(turretConstants.hallEffectChannel);
        this.pidController = this.turretMotor.getClosedLoopController();
        this.config = new SparkMaxConfig();
        this.connectedDebouncer = new Debouncer(0.5);

        // set position factor so we can turn turret to specific angle
        double positionFactor = 360.0 / GEAR_RATIO;
        config.encoder.positionConversionFactor(positionFactor);
        config.encoder.velocityConversionFactor(positionFactor / 60.0);

        // limit turn angle sue to wiring
        config.softLimit
                .forwardSoftLimit(turretConstants.ANGLE_LIMIT)
                .forwardSoftLimitEnabled(true)
                .reverseSoftLimit(-turretConstants.ANGLE_LIMIT)
                .reverseSoftLimitEnabled(true);

        // Set current limit
        config.smartCurrentLimit(20);

        // get and set PID constants
        double[] kPIDs = turretConstants.getPIDs();
        double[] kSVAs = turretConstants.getSVAs();
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
                turretMotor.getEncoder().getPosition(),
                turretMotor.getEncoder().getVelocity(),
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
        return turretMotor.getEncoder().getPosition();
    }

    @Override
    public double getGoalPosition() {
        return pidController.getSetpoint();
    }

    @Override
    public void setAngle(double degrees) {
        double target = Math.max(-turretConstants.ANGLE_LIMIT, Math.min(turretConstants.ANGLE_LIMIT, degrees));
        pidController.setSetpoint(target, ControlType.kPosition);
    }

    @Override
    public void setPID(double kP, double kI, double kD) {
        config.closedLoop.p(kP).i(kI).d(kD);

        this.turretMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void setFF(double kS, double kV, double kA) {
        config.closedLoop.feedForward.kS(kS).kV(kV).kA(kA);

        this.turretMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void stop() {
        turretMotor.stopMotor();
    }
}
