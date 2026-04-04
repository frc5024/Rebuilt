package frc.robot.subsystems.feeder;

import com.revrobotics.sim.SparkFlexSim;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/**
 * 
 */
public class FeederModuleIOSim extends FeederModuleIOSparkMax {
    // Hardware objects
    private final DCMotor dcMotor;
    private final DCMotorSim dcMotorSim;
    private final SparkFlexSim sparkFlexSim;

    private double voltageRequest;

    /**
     * 
     */
    public FeederModuleIOSim() {
        this.dcMotor = DCMotor.getNEO(1);
        this.dcMotorSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(dcMotor, 0.01, GEAR_RATIO), dcMotor);
        this.sparkFlexSim = new SparkFlexSim(this.feederMotor, this.dcMotor);

        this.voltageRequest = 0.0;
    }

    @Override
    public void updateInputs(FeederModuleIOInputs inputs) {
        if (DriverStation.isDisabled()) {
            stop();
        }

        dcMotorSim.update(0.02);

        sparkFlexSim.setPosition(dcMotorSim.getAngularPositionRotations());
        sparkFlexSim.setVelocity(dcMotorSim.getAngularVelocityRPM());

        inputs.data = new FeederModuleIOData(
                true,
                sparkFlexSim.getPosition(),
                sparkFlexSim.getVelocity(),
                voltageRequest,
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

    @Override
    public void set(double voltage) {
        voltageRequest = MathUtil.clamp(voltage * 12, -12.0, 12.0);
        dcMotorSim.setInputVoltage(voltageRequest);
    }
}
