package frc.robot.subsystems.hopper;

import com.revrobotics.sim.SparkMaxSim;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/**
 * 
 */
public class HopperModuleIOSim extends HopperModuleIOSparkMax {
    // Hardware objects
    private final DCMotor dcMotor;
    private final DCMotorSim dcMotorSim;
    private final SparkMaxSim sparkMaxSim;

    private double voltageRequest;

    /**
     * 
     */
    public HopperModuleIOSim() {
        this.dcMotor = DCMotor.getNEO(1);
        this.dcMotorSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(dcMotor, 0.01, GEAR_RATIO), dcMotor);
        this.sparkMaxSim = new SparkMaxSim(this.hopperMotor, this.dcMotor);

        this.voltageRequest = 0.0;
    }

    @Override
    public void updateInputs(HopperModuleIOInputs inputs) {
        if (DriverStation.isDisabled()) {
            stop();
        }

        dcMotorSim.update(0.02);

        sparkMaxSim.setPosition(dcMotorSim.getAngularPositionRotations());
        sparkMaxSim.setVelocity(dcMotorSim.getAngularVelocityRPM());

        inputs.data = new HopperModuleIOData(
                true,
                sparkMaxSim.getPosition(),
                sparkMaxSim.getVelocity(),
                voltageRequest,
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

    @Override
    public void set(double voltage) {
        voltageRequest = MathUtil.clamp(voltage * 12, -12.0, 12.0);
        dcMotorSim.setInputVoltage(voltageRequest);
    }
}
