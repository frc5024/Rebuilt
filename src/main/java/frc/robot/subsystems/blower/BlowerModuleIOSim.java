package frc.robot.subsystems.blower;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class BlowerModuleIOSim implements BlowerModuleIO {
    private final double MOTOR_INTAKE_SPEED = 0.8;
    private final double MOTOR_EJECT_SPEED = -0.6;

    private final DCMotor motor = DCMotor.getKrakenX60Foc(1);
    private final double reduction = (18.0 / 12.0);
    private final double moi = 0.001;

    private final DCMotorSim dcMotorSim;

    private double appliedVoltage = 0.0;

    /**
     * 
     */
    public BlowerModuleIOSim() {
        this.dcMotorSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(this.motor, this.moi, this.reduction),
                this.motor);
    }

    @Override
    public void updateInputs(BlowerModuleIOInputs inputs) {
        if (DriverStation.isDisabled()) {
            stop();
        }

        this.dcMotorSim.update(0.02);

        inputs.data = new BlowerModuleIOData(
                true,
                this.dcMotorSim.getAngularPositionRad(),
                this.dcMotorSim.getAngularVelocityRadPerSec(),
                this.appliedVoltage,
                0.0,
                this.dcMotorSim.getCurrentDrawAmps(),
                0.0);
    }

    @Override
    public void start(double speed) {
        this.appliedVoltage = MathUtil.clamp(MOTOR_EJECT_SPEED * 12, -12.0, 12.0);
        this.dcMotorSim.setInputVoltage(this.appliedVoltage);
    }

    @Override
    public void stop() {
        this.appliedVoltage = 0.0;
        this.dcMotorSim.setInputVoltage(this.appliedVoltage);
    }
}