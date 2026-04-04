package frc.robot.subsystems.hopper;

import com.revrobotics.sim.SparkMaxSim;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/**
 * 
 */
public class HopperModuleIOSim extends HopperModuleIOSparkMaxRelativeEncoder {
    // Hardware objects
    private final DCMotor dcMotor;
    private final DCMotorSim dcMotorSim;
    private final SparkMaxSim sparkMaxSim;

    /**
     * 
     */
    public HopperModuleIOSim() {
        this.dcMotor = DCMotor.getNEO(1);
        this.dcMotorSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(dcMotor, 0.001, GEAR_RATIO), dcMotor);
        this.sparkMaxSim = new SparkMaxSim(this.hopperMotor, this.dcMotor);
    }

    @Override
    public void updateInputs(HopperModuleIOInputs inputs) {
        if (DriverStation.isDisabled()) {
            stop();
        }

        double appliedVoltage = sparkMaxSim.getAppliedOutput() * RobotController.getBatteryVoltage();
        dcMotorSim.setInput(appliedVoltage);
        dcMotorSim.update(0.02);

        sparkMaxSim.iterate(dcMotorSim.getAngularVelocityRPM(), RobotController.getBatteryVoltage(), 0.02);

        sparkMaxSim.setMotorCurrent(dcMotorSim.getCurrentDrawAmps());
        sparkMaxSim.setBusVoltage(appliedVoltage);

        inputs.data = new HopperModuleIOData(
                true,
                sparkMaxSim.getPosition(),
                sparkMaxSim.getVelocity() * GEAR_RATIO,
                appliedVoltage,
                0.0,
                sparkMaxSim.getMotorCurrent(),
                0.0);
    }

    @Override
    public double getCurrentDrawAmps() {
        return sparkMaxSim.getMotorCurrent();
    }

    @Override
    public double getPosition() {
        return sparkMaxSim.getPosition();
    }
}
