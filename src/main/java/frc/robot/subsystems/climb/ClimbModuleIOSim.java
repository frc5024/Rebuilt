package frc.robot.subsystems.climb;

import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;

/**
 * 
 */
public class ClimbModuleIOSim extends ClimbModuleIOTalonFX {
    private final TalonFXSimState talonFXSimState;

    /**
     * 
     */
    public ClimbModuleIOSim() {
        this.talonFXSimState = this.climbMotor.getSimState();
        this.talonFXSimState.Orientation = ChassisReference.CounterClockwise_Positive;
    }

    @Override
    public void updateInputs(ClimbModuleIOInputs inputs) {
        if (DriverStation.isDisabled()) {
            stop();
        }

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
}
