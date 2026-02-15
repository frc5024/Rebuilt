package frc.robot.subsystems.blower;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class BlowerModuleIOSim implements BlowerModuleIO {
    // Hardware objects
    private final DCMotor motor = DCMotor.getKrakenX60Foc(1);
    private final DCMotorSim dcMotorSim;

    private final double reduction = (18.0 / 12.0);
    private final double moi = 0.001;

    // Variables for ramping the motor
    protected double voltageRequest;
    protected double startTime;
    protected boolean isRamping;


    /**
     * 
     */
    public BlowerModuleIOSim() {
        this.dcMotorSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(this.motor, this.moi, this.reduction),
                this.motor);

        this.voltageRequest = 0.0;
        this.startTime = 0.0;
        this.isRamping = false;
    }

    @Override
    public void updateInputs(BlowerModuleIOInputs inputs) {
        if (DriverStation.isDisabled()) {
            stop();
        }

        if (this.isRamping) {
            double elapsedTime = Timer.getFPGATimestamp() - this.startTime;
            this.voltageRequest = MathUtil.clamp((TARGET_VOLTAGE / RAMP_TIME_SEC) * elapsedTime, -TARGET_VOLTAGE, TARGET_VOLTAGE);

            this.dcMotorSim.setInputVoltage(this.voltageRequest);
        }

        this.dcMotorSim.update(0.02);

        inputs.data = new BlowerModuleIOData(
                true,
                this.dcMotorSim.getAngularPositionRad(),
                this.dcMotorSim.getAngularVelocityRadPerSec(),
                this.voltageRequest,
                0.0,
                this.dcMotorSim.getCurrentDrawAmps(),
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
        this.voltageRequest = 0.0;
        this.dcMotorSim.setInputVoltage(this.voltageRequest);
    }
}