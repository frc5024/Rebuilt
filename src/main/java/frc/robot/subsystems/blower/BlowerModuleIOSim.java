package frc.robot.subsystems.blower;

import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class BlowerModuleIOSim extends BlowerModuleIOTalonFX {
    // Hardware objects
    private final DCMotor dcMotor;
    private final DCMotorSim dcMotorSim;
    private final TalonFXSimState talonFXSimState;

    // Variables for ramping the motor
    protected double voltageRequest;

    /**
     * 
     */
    public BlowerModuleIOSim() {
        this.dcMotor = DCMotor.getFalcon500(1);
        this.dcMotorSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(this.dcMotor, 0.001, 1.0),
                this.dcMotor);
        this.talonFXSimState = this.blowerTalon.getSimState();
        this.talonFXSimState.Orientation = ChassisReference.Clockwise_Positive;

        this.voltageRequest = 0.0;
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
            this.voltageRequest = MathUtil.clamp((TARGET_VOLTAGE / RAMP_TIME_SEC) *
                    elapsedTime, -TARGET_VOLTAGE, TARGET_VOLTAGE);
            // voltageRequest = talonFXSimState.getMotorVoltage();
            dcMotorSim.setInputVoltage(voltageRequest);
        }

        dcMotorSim.update(0.02);
        talonFXSimState.setRawRotorPosition(dcMotorSim.getAngularPositionRotations());
        talonFXSimState.setRotorVelocity(dcMotorSim.getAngularVelocityRPM());
        talonFXSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

        inputs.data = new BlowerModuleIOData(
                true,
                blowerTalon.getPosition().getValueAsDouble(),
                blowerTalon.getVelocity().getValueAsDouble(),
                voltageRequest,
                0.0,
                talonFXSimState.getSupplyCurrent(),
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
        voltageRequest = 0.0;
        dcMotorSim.setInputVoltage(voltageRequest);
    }
}