package frc.robot.subsystems.blower;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.generated.TunerConstants;

public class BlowerModuleIOTalonFX implements BlowerModuleIO {
    // Hardware objects
    private final TalonFX blowerTalon;
    private final int MOTOR_ID = 18;

    // Inputs from drive motor
    private final StatusSignal<Angle> drivePosition;
    private final StatusSignal<AngularVelocity> driveVelocity;
    private final StatusSignal<Voltage> driveAppliedVolts;
    private final StatusSignal<Current> driveCurrent;

    // Connection debouncers
    private final Debouncer driveConnectedDebounce = new Debouncer(0.5);

    // Create a control request object for Percent Output (Duty Cycle)
    private final MotionMagicVoltage voltageRequest = new MotionMagicVoltage(0);

    /**
     * 
     */
    public BlowerModuleIOTalonFX() {
        this.blowerTalon = new TalonFX(MOTOR_ID, TunerConstants.DrivetrainConstants.CANBusName);
        
        TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();
        talonFXConfiguration.Slot0.kS = 0.25;
        talonFXConfiguration.Slot0.kV = 0.12;
        talonFXConfiguration.Slot0.kG = 0.01;
        talonFXConfiguration.Slot0.kP = 4.8;
        talonFXConfiguration.Slot0.kI = 0.0;
        talonFXConfiguration.Slot0.kD = 0.1;

        talonFXConfiguration.MotionMagic.MotionMagicCruiseVelocity = 80;
        talonFXConfiguration.MotionMagic.MotionMagicAcceleration = 160;
        talonFXConfiguration.MotionMagic.MotionMagicJerk = 1600;

        tryUntilOk(5, () -> this.blowerTalon.getConfigurator().apply(talonFXConfiguration, 0.25));
        tryUntilOk(5, () -> this.blowerTalon.setPosition(0.0, 0.25));

        // Create drive status signals
        this.drivePosition = this.blowerTalon.getPosition();
        this.driveVelocity = this.blowerTalon.getVelocity();
        this.driveAppliedVolts = this.blowerTalon.getMotorVoltage();
        this.driveCurrent = this.blowerTalon.getStatorCurrent();

        // Configure periodic frames
        BaseStatusSignal.setUpdateFrequencyForAll(SwerveDriveConstants.ODOMETRY_FREQUENCY, this.drivePosition);
        BaseStatusSignal.setUpdateFrequencyForAll(
                50.0,
                this.driveVelocity,
                this.driveAppliedVolts,
                this.driveCurrent);
        ParentDevice.optimizeBusUtilizationForAll(this.blowerTalon);
    }

    @Override
    public void updateInputs(BlowerModuleIOInputs inputs) {
        if (DriverStation.isDisabled()) {
            stop();
        }
        // Refresh all signals
        var driveStatus = BaseStatusSignal.refreshAll(this.drivePosition, this.driveVelocity, this.driveAppliedVolts, this.driveCurrent);

        // Update drive inputs
        inputs.data = new BlowerModuleIOData(
                driveConnectedDebounce.calculate(driveStatus.isOK()),
                Units.rotationsToRadians(this.drivePosition.getValueAsDouble()),
                Units.rotationsToRadians(this.driveVelocity.getValueAsDouble()),
                this.driveAppliedVolts.getValueAsDouble(),
                0.0,
                this.driveCurrent.getValueAsDouble(),
                0.0);
    }

    @Override
    public boolean isRunning() {
        return this.blowerTalon.getVelocity().getValueAsDouble() != 0.0;
    }

    @Override
    public void start() {
        this.blowerTalon.setControl(voltageRequest.withPosition(100));
    }

    @Override
    public void stop() {
        this.blowerTalon.setControl(voltageRequest.withPosition(0.0));
    }
}