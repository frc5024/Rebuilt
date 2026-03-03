package frc.robot.subsystems.shooter;

import com.revrobotics.sim.SparkFlexSim;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

/**
 * 
 */
public class ShooterModuleIOSim extends ShooterModuleIOSparkFlex2 {
    private final DCMotor dcMotor;
    private final SparkFlexSim sparkFlexSim;
    private final FlywheelSim flywheelSim;

    private double voltageRequest;

    /**
     * 
     */
    public ShooterModuleIOSim() {
        this.dcMotor = DCMotor.getNeoVortex(1);
        this.flywheelSim = new FlywheelSim(
                LinearSystemId.createFlywheelSystem(DCMotor.getNeoVortex(2), 0.003, GEAR_RATIO), dcMotor);
        this.sparkFlexSim = new SparkFlexSim(this.leadMotor, this.dcMotor);

        this.voltageRequest = 0.0;
    }

    @Override
    public void updateInputs(ShooterModuleIOInputs inputs) {
        if (DriverStation.isDisabled()) {
            stop();
        }

        flywheelSim.setInput(sparkFlexSim.getAppliedOutput() * RobotController.getBatteryVoltage());
        flywheelSim.update(0.02);

        double velocityRPM = Units.radiansPerSecondToRotationsPerMinute(flywheelSim.getAngularVelocityRadPerSec());
        sparkFlexSim.iterate(velocityRPM, RobotController.getBatteryVoltage(), 0.020);

        sparkFlexSim.setMotorCurrent(flywheelSim.getCurrentDrawAmps());
        sparkFlexSim.setBusVoltage(voltageRequest);

        inputs.data = new ShooterModuleIOData(
                true,
                sparkFlexSim.getPosition(),
                sparkFlexSim.getVelocity(),
                sparkFlexSim.getAppliedOutput() * RobotController.getBatteryVoltage(),
                0.0,
                sparkFlexSim.getMotorCurrent(),
                0.0,
                true,
                sparkFlexSim.getPosition(),
                sparkFlexSim.getVelocity(),
                sparkFlexSim.getAppliedOutput() * RobotController.getBatteryVoltage(),
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

    // @Override
    // public double getVelocity() {
    // return flywheelSim.getAngularVelocityRadPerSec();
    // }

    @Override
    public void set(double voltage) {
        voltageRequest = MathUtil.clamp(voltage * 12, -12.0, 12.0);
        flywheelSim.setInputVoltage(voltageRequest);
    }

    @Override
    public void setVoltage(double voltage) {
        voltageRequest = MathUtil.clamp(voltage, -12.0, 12.0);
        flywheelSim.setInputVoltage(voltageRequest);
    }
}
