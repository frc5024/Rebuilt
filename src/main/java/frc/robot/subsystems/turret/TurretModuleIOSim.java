package frc.robot.subsystems.turret;

import com.revrobotics.sim.SparkMaxSim;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import frc.robot.Constants.TurretConstants;

/**
 * 
 */
public class TurretModuleIOSim extends TurretModuleIOSparkMaxClosedLoopEncoder {
    // Hardware objects
    private final DCMotor dcMotor;
    private final DCMotorSim dcMotorSim;
    private final SparkMaxSim sparkMaxSim;
    // private final DutyCycleEncoderSim absoluteEncoderSim;
    private final DIOSim hallEffectSim;

    /**
     * 
     */
    public TurretModuleIOSim() {
        this.dcMotor = DCMotor.getNeo550(1);
        this.dcMotorSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(dcMotor, 0.01, 4.0), dcMotor);
        this.sparkMaxSim = new SparkMaxSim(this.turretMotor, this.dcMotor);

        // this.absoluteEncoderSim = new DutyCycleEncoderSim(absoluteEncoder);
        this.hallEffectSim = new DIOSim(hallEffectSensor);
    }

    @Override
    public void updateInputs(TurretModuleIOInputs inputs) {
        if (DriverStation.isDisabled()) {
            stop();
        }

        double appliedVoltage = sparkMaxSim.getAppliedOutput() * RobotController.getBatteryVoltage();
        dcMotorSim.setInput(appliedVoltage);
        dcMotorSim.update(0.02);

        double velocityRPM = Units.radiansPerSecondToRotationsPerMinute(dcMotorSim.getAngularVelocityRadPerSec());
        sparkMaxSim.iterate(velocityRPM, RobotController.getBatteryVoltage(), 0.020);

        sparkMaxSim.setMotorCurrent(dcMotorSim.getCurrentDrawAmps());
        sparkMaxSim.setBusVoltage(appliedVoltage);

        // absoluteEncoderSim.set(getCurrentAngle());

        // real hall effect returns false when homed
        boolean isNearHome = Math.abs(getCurrentAngle() - TurretConstants.ANGLE_LIMIT) < 2.0;
        hallEffectSim.setValue(!isNearHome);

        inputs.data = new TurretModuleIOData(
                true,
                sparkMaxSim.getPosition(),
                sparkMaxSim.getVelocity(),
                appliedVoltage,
                0.0,
                sparkMaxSim.getMotorCurrent(),
                0.0,
                !hallEffectSim.getValue());
    }

    @Override
    public double getCurrentDrawAmps() {
        return sparkMaxSim.getMotorCurrent();
    }

    @Override
    public boolean getHallEffectValue() {
        return hallEffectSim.getValue();
    }
}
