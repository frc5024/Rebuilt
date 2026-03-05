package frc.robot.subsystems.blower;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.generated.TunerConstants;

public class BlowerModuleIOTalonFX implements BlowerModuleIO {
    // Hardware objects
    protected final TalonFX blowerTalon;
    private final int MOTOR_ID = 18;

    // Inputs from drive motor
    private final StatusSignal<Angle> drivePosition;
    private final StatusSignal<AngularVelocity> driveVelocity;
    private final StatusSignal<Voltage> driveAppliedVolts;
    private final StatusSignal<Current> driveCurrent;

    // Connection debouncers
    private final Debouncer driveConnectedDebounce = new Debouncer(0.5);

    // Variables for ramping the motor
    protected VoltageOut voltageRequest;
    protected double startTime;
    protected boolean isRamping;

    /**
     * 
     */
    public BlowerModuleIOTalonFX() {
        this.blowerTalon = new TalonFX(MOTOR_ID, TunerConstants.DrivetrainConstants.CANBusName);

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

        this.voltageRequest = new VoltageOut(0);
        this.startTime = 0.0;
        this.isRamping = false;
    }

    @Override
    public void updateInputs(BlowerModuleIOInputs inputs) {
        if (DriverStation.isDisabled()) {
            stop();
        }

        if (isRamping) {
            double elapsedTime = Timer.getFPGATimestamp() - this.startTime;
            double targetVoltage = MathUtil.clamp((TARGET_VOLTAGE / RAMP_TIME_SEC) * elapsedTime, -TARGET_VOLTAGE,
                    TARGET_VOLTAGE);

            blowerTalon.setControl(voltageRequest.withOutput(targetVoltage));
        }

        // Refresh all signals
        var driveStatus = BaseStatusSignal.refreshAll(drivePosition, driveVelocity, driveAppliedVolts,
                driveCurrent);

        // Update drive inputs
        inputs.data = new BlowerModuleIOData(
                driveConnectedDebounce.calculate(driveStatus.isOK()),
                Units.rotationsToRadians(drivePosition.getValueAsDouble()),
                Units.rotationsToRadians(driveVelocity.getValueAsDouble()),
                driveAppliedVolts.getValueAsDouble(),
                0.0,
                driveCurrent.getValueAsDouble(),
                0.0);
    }

    @Override
    public boolean isRunning() {
        return isRamping;
    }

    @Override
    public void start() {
        isRamping = true;
        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public void stop() {
        isRamping = false;
        blowerTalon.setControl(voltageRequest.withOutput(0.0));
    }
}