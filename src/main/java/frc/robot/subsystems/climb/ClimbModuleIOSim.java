package frc.robot.subsystems.climb;

import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/**
 * 
 */
public class ClimbModuleIOSim extends ClimbModuleIOTalonFX {
    private final DCMotor dcMotor;
    private final DCMotorSim dcMotorSim;
    private final TalonFXSimState talonFXSimState;

    private double voltageRequest;

    /**
     * 
     */
    public ClimbModuleIOSim() {
        this.dcMotor = DCMotor.getFalcon500(1);
        this.dcMotorSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(dcMotor, 0.001, 4.0), dcMotor);
        this.talonFXSimState = this.climbMotor.getSimState();
        this.talonFXSimState.Orientation = ChassisReference.CounterClockwise_Positive;

        this.voltageRequest = 0.0;
    }

    @Override
    public void updateInputs(ClimbModuleIOInputs inputs) {
        if (DriverStation.isDisabled()) {
            stop();
        }

        if (climbMotor.getPosition().getValueAsDouble() >= 0.0
                || climbMotor.getPosition().getValueAsDouble() <= -0.25) {
            stop();
        }

        double motorVoltage = talonFXSimState.getMotorVoltage();
        dcMotorSim.setInputVoltage(motorVoltage);
        dcMotorSim.update(0.02);

        talonFXSimState.setRawRotorPosition(dcMotorSim.getAngularPositionRotations());
        talonFXSimState.setRotorVelocity(dcMotorSim.getAngularVelocityRPM() / 60.0);
        talonFXSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

        inputs.data = new ClimbModuleIOData(
                true,
                climbMotor.getPosition().getValueAsDouble(),
                climbMotor.getVelocity().getValueAsDouble(),
                talonFXSimState.getMotorVoltage(),
                0.0,
                talonFXSimState.getSupplyCurrent(),
                0.0);
    }

    @Override
    public double getCurrentDrawAmps() {
        return talonFXSimState.getSupplyCurrent();
    }

    @Override
    public double getPosition() {
        return climbMotor.getPosition().getValueAsDouble();
    }
}
