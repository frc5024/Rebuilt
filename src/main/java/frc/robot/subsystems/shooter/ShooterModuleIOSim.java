package frc.robot.subsystems.shooter;

import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

/**
 * 
 */
public class ShooterModuleIOSim extends ShooterModuleIOSparkFlexClosedLoopController {
    private final DCMotor dcMotor;
    private final FlywheelSim flywheelSim;
    private final SparkFlexSim sparkFlexSim;
    private final SparkRelativeEncoderSim rollerEncoderSim;

    /**
     * 
     */
    public ShooterModuleIOSim() {
        this.dcMotor = DCMotor.getNeoVortex(1);
        this.flywheelSim = new FlywheelSim(
                LinearSystemId.createFlywheelSystem(DCMotor.getNeoVortex(2), 0.01, GEAR_RATIO), dcMotor);
        this.sparkFlexSim = new SparkFlexSim(this.leadMotor, this.dcMotor);
        this.rollerEncoderSim = new SparkRelativeEncoderSim(leadMotor);
    }

    @Override
    public void updateInputs(ShooterModuleIOInputs inputs) {
        if (DriverStation.isDisabled()) {
            stop();
        }

        double appliedVoltage = sparkFlexSim.getAppliedOutput() * RobotController.getBatteryVoltage();
        flywheelSim.setInput(appliedVoltage);
        flywheelSim.update(0.02);

        double velocityRPM = Units.radiansPerSecondToRotationsPerMinute(flywheelSim.getAngularVelocityRadPerSec());
        sparkFlexSim.iterate(velocityRPM, RobotController.getBatteryVoltage(), 0.020);

        sparkFlexSim.setMotorCurrent(flywheelSim.getCurrentDrawAmps());
        sparkFlexSim.setBusVoltage(appliedVoltage);

        inputs.data = new ShooterModuleIOData(
                true,
                sparkFlexSim.getPosition(),
                sparkFlexSim.getVelocity(),
                appliedVoltage,
                0.0,
                sparkFlexSim.getMotorCurrent(),
                0.0,
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
}
