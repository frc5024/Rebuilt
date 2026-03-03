package frc.robot.subsystems.turret;

import com.revrobotics.sim.SparkMaxSim;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/**
 * 
 */
public class TurretModuleIOSim extends TurretModuleIOSparkMax2 {
    // Hardware objects
    private final DCMotor dcMotor;
    private final DCMotorSim dcMotorSim;
    private final SparkMaxSim sparkMaxSim;

    private double voltageRequest;

    /**
     * 
     */
    public TurretModuleIOSim() {
        this.dcMotor = DCMotor.getNEO(1);
        this.dcMotorSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(dcMotor, 0.01, 4.0), dcMotor);
        this.sparkMaxSim = new SparkMaxSim(this.turretMotor, this.dcMotor);

        this.voltageRequest = 0.0;
    }

    @Override
    public void updateInputs(TurretModuleIOInputs inputs) {
        if (DriverStation.isDisabled()) {
            stop();
        }

        dcMotorSim.setInput(sparkMaxSim.getAppliedOutput() * RobotController.getBatteryVoltage());
        dcMotorSim.update(0.02);

        double velocityRPM = Units.radiansPerSecondToRotationsPerMinute(dcMotorSim.getAngularVelocityRadPerSec());
        sparkMaxSim.iterate(velocityRPM, RobotController.getBatteryVoltage(), 0.020);

        sparkMaxSim.setMotorCurrent(dcMotorSim.getCurrentDrawAmps());
        sparkMaxSim.setBusVoltage(voltageRequest);

        inputs.data = new TurretModuleIOData(
                true,
                sparkMaxSim.getPosition(),
                sparkMaxSim.getVelocity(),
                sparkMaxSim.getAppliedOutput() * RobotController.getBatteryVoltage(),
                0.0,
                sparkMaxSim.getMotorCurrent(),
                0.0);
    }

    @Override
    public double getCurrentDrawAmps() {
        return sparkMaxSim.getMotorCurrent();
    }
}
