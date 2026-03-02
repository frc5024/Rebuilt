package frc.robot.subsystems.turret;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.turretConstants;

public class TurretModuleIOSparkMax2 implements TurretModuleIO {
    /* Constants */
    protected final double GEAR_RATIO = 4.0;
    protected final double ANGLE_LIMIT = 30.0;

    /* Hardware */
    protected final SparkMax turretMotor;
    private final SparkClosedLoopController pidController;

    /* Connection debouncers */
    private final Debouncer connectedDebouncer;

    /**
     * 
     */
    public TurretModuleIOSparkMax2() {
        this.turretMotor = new SparkMax(turretConstants.turretMotorChannel, SparkLowLevel.MotorType.kBrushless);
        this.pidController = this.turretMotor.getClosedLoopController();
        this.connectedDebouncer = new Debouncer(0.5);

        // set position factor so we can turn turret to specific angle
        SparkMaxConfig config = new SparkMaxConfig();
        double positionFactor = 360.0 / GEAR_RATIO;
        config.encoder.positionConversionFactor(positionFactor);
        config.encoder.velocityConversionFactor(positionFactor / 60.0);

        // limit turn angle sue to wiring
        config.softLimit
                .forwardSoftLimit(ANGLE_LIMIT)
                .forwardSoftLimitEnabled(true)
                .reverseSoftLimit(-ANGLE_LIMIT)
                .reverseSoftLimitEnabled(true);

        // set controller PID
        config.closedLoop.p(0.1);
        config.closedLoop.i(0.0);
        config.closedLoop.d(0.005);

        this.turretMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(TurretModuleIOInputs inputs) {
        if (DriverStation.isDisabled()) {
            stop();
        }

        inputs.data = new TurretModuleIOData(
                connectedDebouncer.calculate(turretMotor.getFaults().other),
                turretMotor.getEncoder().getPosition(),
                turretMotor.getEncoder().getVelocity(),
                turretMotor.getAppliedOutput(),
                0.0,
                turretMotor.getOutputCurrent(),
                turretMotor.getMotorTemperature());
    }

    @Override
    public double getCurrentDrawAmps() {
        return turretMotor.getOutputCurrent();
    }

    @Override
    public double getAngle() {
        return turretMotor.getEncoder().getPosition();
    }

    @Override
    public double getSetpoint() {
        return pidController.getSetpoint();
    }

    @Override
    public boolean isAtSetpoint() {
        return pidController.isAtSetpoint();
    }

    @Override
    public void setAngle(double degrees) {
        double target = Math.max(-ANGLE_LIMIT, Math.min(ANGLE_LIMIT, degrees));
        pidController.setSetpoint(target, ControlType.kPosition);
    }

    @Override
    public void stop() {
        turretMotor.stopMotor();
    }
}
