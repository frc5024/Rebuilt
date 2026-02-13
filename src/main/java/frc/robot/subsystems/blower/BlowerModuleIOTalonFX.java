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
    private final VoltageOut voltageRequest = new VoltageOut(0);
    private final double TARGET_VOLTAGE = 12.0;
    private double startTime = 0.0;
    private boolean isRamping = false;

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
    }

    @Override
    public void updateInputs(BlowerModuleIOInputs inputs) {
        if (DriverStation.isDisabled()) {
            stop();
        }

        if (this.isRamping) {
            double elapsedTime = Timer.getFPGATimestamp() - this.startTime;
            double targetVoltage = MathUtil.clamp((TARGET_VOLTAGE / 10.0) * elapsedTime, -TARGET_VOLTAGE,
                    TARGET_VOLTAGE);

            this.blowerTalon.setControl(voltageRequest.withOutput(targetVoltage));
        }

        // Refresh all signals
        var driveStatus = BaseStatusSignal.refreshAll(this.drivePosition, this.driveVelocity, this.driveAppliedVolts,
                this.driveCurrent);

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
        return this.isRamping;
    }

    @Override
    public void start() {
        this.isRamping = true;
        this.startTime = Timer.getFPGATimestamp();
    }

    @Override
    public void stop() {
        this.isRamping = false;
        this.blowerTalon.setControl(voltageRequest.withOutput(0.0));
    }
}