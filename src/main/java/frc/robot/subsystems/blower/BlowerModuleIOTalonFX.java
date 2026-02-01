package frc.robot.subsystems.blower;

import java.util.Queue;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;
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
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.util.PhoenixOdometryThread;

public class BlowerModuleIOTalonFX implements BlowerModuleIO {
    // Hardware objects
    private final TalonFX blowerTalon;
    private final int MOTOR_ID = 18;

    // Inputs from drive motor
    private final StatusSignal<Angle> drivePosition;
    private final Queue<Double> drivePositionQueue;
    private final StatusSignal<AngularVelocity> driveVelocity;
    private final StatusSignal<Voltage> driveAppliedVolts;
    private final StatusSignal<Current> driveCurrent;

    // Connection debouncers
    private final Debouncer driveConnectedDebounce = new Debouncer(0.5);

    // Create a control request object for Percent Output (Duty Cycle)
    private final DutyCycleOut m_dutyCycleOut = new DutyCycleOut(0);
    private double appliedVoltage = 0.0;

    /**
     * 
     */
    public BlowerModuleIOTalonFX() {
        this.blowerTalon = new TalonFX(MOTOR_ID, TunerConstants.DrivetrainConstants.CANBusName);

        // Create drive status signals
        drivePosition = blowerTalon.getPosition();
        drivePositionQueue = PhoenixOdometryThread.getInstance().registerSignal(blowerTalon.getPosition());
        driveVelocity = blowerTalon.getVelocity();
        driveAppliedVolts = blowerTalon.getMotorVoltage();
        driveCurrent = blowerTalon.getStatorCurrent();

        // Configure periodic frames
        BaseStatusSignal.setUpdateFrequencyForAll(SwerveDriveConstants.ODOMETRY_FREQUENCY, drivePosition);
        BaseStatusSignal.setUpdateFrequencyForAll(
                50.0,
                driveVelocity,
                driveAppliedVolts,
                driveCurrent);
        ParentDevice.optimizeBusUtilizationForAll(blowerTalon);
    }

    @Override
    public void updateInputs(BlowerModuleIOInputs inputs) {
        if (DriverStation.isDisabled()) {
            stop();
        }
        // Refresh all signals
        var driveStatus = BaseStatusSignal.refreshAll(drivePosition, driveVelocity, driveAppliedVolts, driveCurrent);

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
    public void start(double speed) {
        this.blowerTalon.setControl(m_dutyCycleOut.withOutput(speed));
    }

    @Override
    public void stop() {
        this.blowerTalon.setControl(m_dutyCycleOut.withOutput(0.0));
    }
}