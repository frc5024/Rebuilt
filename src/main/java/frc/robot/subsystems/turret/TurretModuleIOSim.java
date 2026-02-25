package frc.robot.subsystems.turret;

import com.revrobotics.sim.SparkMaxSim;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/**
 * 
 */
public class TurretModuleIOSim extends TurretModuleIOSparkMax {
    // Hardware objects
    private final DCMotor dcMotor;
    private final SparkMaxSim sparkMaxSim;
    private final DCMotorSim dcMotorSim;

    private double voltageRequest;

    /**
     * 
     */
    public TurretModuleIOSim() {
        this.dcMotor = DCMotor.getNEO(1);
        this.sparkMaxSim = new SparkMaxSim(this.turretMotor, this.dcMotor);
        this.dcMotorSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(dcMotor, 0.01, 4.0), dcMotor);

        this.voltageRequest = 0.0;
    }

    @Override
    public void updateInputs(TurretModuleIOInputs inputs) {
        if (DriverStation.isDisabled()) {
            stop();
        }

        dcMotorSim.update(0.02);

        sparkMaxSim.setPosition(dcMotorSim.getAngularPositionRotations());
        sparkMaxSim.setVelocity(dcMotorSim.getAngularVelocityRPM());

        inputs.data = new TurretModuleIOData(
                true,
                sparkMaxSim.getPosition(),
                sparkMaxSim.getVelocity(),
                voltageRequest,
                0.0,
                sparkMaxSim.getMotorCurrent(),
                0.0);
    }
}
