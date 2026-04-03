package frc.robot.subsystems.intake;

import com.revrobotics.sim.SparkFlexSim;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/**
 * 
 */
public class RollerModuleIOSim extends RollerModuleIOSparkFlexClosedLoopController {
    // Hardware objects
    private final DCMotor dcMotor;
    private final DCMotorSim dcMotorSim;
    private final SparkFlexSim sparkFlexSim;

    /**
     * 
     */
    public RollerModuleIOSim() {
        this.dcMotor = DCMotor.getNeoVortex(1);
        this.dcMotorSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(dcMotor, 0.01, GEAR_RATIO), dcMotor);
        this.sparkFlexSim = new SparkFlexSim(this.rollerMotor, this.dcMotor);
    }

    @Override
    public void updateInputs(RollerModuleIOInputs inputs) {
        if (DriverStation.isDisabled()) {
            stop();
        }

        double appliedVoltage = sparkFlexSim.getAppliedOutput() * RobotController.getBatteryVoltage();
        dcMotorSim.setInput(appliedVoltage);
        dcMotorSim.update(0.02);

        sparkFlexSim.iterate(dcMotorSim.getAngularVelocityRPM(), RobotController.getBatteryVoltage(), 0.020);

        sparkFlexSim.setMotorCurrent(dcMotorSim.getCurrentDrawAmps());
        sparkFlexSim.setBusVoltage(appliedVoltage);

        inputs.data = new RollerModuleIOData(
                true,
                sparkFlexSim.getPosition(),
                sparkFlexSim.getVelocity(),
                appliedVoltage,
                0.0,
                sparkFlexSim.getMotorCurrent(),
                0.0);
    }

    @Override
    public double getCurrentDrawAmps() {
        return sparkFlexSim.getMotorCurrent();
    }

    @Override
    public double getPosition() {
        return sparkFlexSim.getPosition();
    }
}
