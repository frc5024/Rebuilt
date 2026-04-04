package frc.robot.subsystems.climb;

import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants.ClimbConstants;

/**
 * 
 */
public class ClimbModuleIOSim extends ClimbModuleIOTalonFX {
    private final DCMotor dcMotor;
    private final ElevatorSim elevatorSim;
    private final TalonFXSimState talonFXSimState;

    /**
     * 
     */
    public ClimbModuleIOSim() {
        this.dcMotor = DCMotor.getFalcon500(1);
        this.elevatorSim = new ElevatorSim(LinearSystemId.createDCMotorSystem(dcMotor, 0.01, GEAR_RATIO), dcMotor, 0.0,
                Units.inchesToMeters(ClimbConstants.EXTEND_LENGTH_INCHES), false, 0.0);
        this.talonFXSimState = this.climbMotor.getSimState();
        this.talonFXSimState.Orientation = ChassisReference.CounterClockwise_Positive;
    }

    @Override
    public void updateInputs(ClimbModuleIOInputs inputs) {
        if (DriverStation.isDisabled()) {
            stop();
        }

        elevatorSim.setInput(talonFXSimState.getMotorVoltage());
        elevatorSim.update(0.02);

        double mechanismRotations = elevatorSim.getPositionMeters() / (2.0 * DRUM_RADIUS * Math.PI);
        double motorRotations = mechanismRotations * GEAR_RATIO;
        double motorVelocityRPS = (elevatorSim.getVelocityMetersPerSecond() / (2.0 * DRUM_RADIUS * Math.PI))
                * GEAR_RATIO;

        if (motorRotations < 0.0) {
            motorRotations = 0.0;
            motorVelocityRPS = 0.0;
        }

        talonFXSimState.setRawRotorPosition(motorRotations);
        talonFXSimState.setRotorVelocity(motorVelocityRPS);
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
}
