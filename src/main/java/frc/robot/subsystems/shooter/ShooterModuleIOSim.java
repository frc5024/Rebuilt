package frc.robot.subsystems.shooter;

import com.revrobotics.sim.SparkFlexSim;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

/**
 * 
 */
public class ShooterModuleIOSim extends ShooterModuleIOSparkFlex {
    private final DCMotor dcMotor;
    private final SparkFlexSim sparkFlexSim;
    private final FlywheelSim flywheelSim;

    private double voltageRequest;

    /**
     * 
     */
    public ShooterModuleIOSim() {
        this.dcMotor = DCMotor.getNEO(1);
        this.sparkFlexSim = new SparkFlexSim(this.flywheel1, this.dcMotor);
        this.flywheelSim = new FlywheelSim(LinearSystemId.createFlywheelSystem(dcMotor, 0.003, 1.0), dcMotor);

        this.voltageRequest = 0.0;
    }

    @Override
    public double getPosition() {
        return sparkFlexSim.getPosition();
    }

    @Override
    public double getVelocity() {
        return flywheelSim.getAngularVelocityRPM();
    }

    @Override
    public void set(double voltage) {
        voltageRequest = MathUtil.clamp(voltage * 12, -12.0, 12.0);
        flywheelSim.setInputVoltage(voltageRequest);
    }

    @Override
    public void setVoltage(double voltage) {
        voltageRequest = MathUtil.clamp(voltage * 12, -12.0, 12.0);
        flywheelSim.setInputVoltage(voltageRequest);
    }

    @Override
    public void updateInputs(ShooterModuleIOInputs inputs) {
        if (DriverStation.isDisabled()) {
            stop();
        }

        flywheelSim.update(0.02);
        sparkFlexSim.setVelocity(flywheelSim.getAngularVelocityRPM());
        double deltaRotations = (flywheelSim.getAngularVelocityRPM() / 60.0) * 0.02;
        sparkFlexSim.setPosition(sparkFlexSim.getPosition() + deltaRotations);

        sparkFlexSim.setMotorCurrent(flywheelSim.getCurrentDrawAmps());
        sparkFlexSim.setBusVoltage(voltageRequest);

        inputs.data = new ShooterModuleIOData(
                true,
                sparkFlexSim.getPosition(),
                sparkFlexSim.getVelocity(),
                voltageRequest,
                0.0,
                sparkFlexSim.getMotorCurrent(),
                0.0,
                true,
                sparkFlexSim.getPosition(),
                sparkFlexSim.getVelocity(),
                voltageRequest,
                0.0,
                sparkFlexSim.getMotorCurrent(),
                0.0);
    }
}
