package frc.robot.subsystems.intake;

import com.revrobotics.sim.SparkMaxSim;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import frc.robot.Constants.IntakeConstants.ArmConstants;

/**
 * 
 */
public class ArmModuleIOSim extends ArmModuleIOSparkMaxRelativeEncoder {
    // Hardware objects
    private final DCMotor armDcMotor;
    private final DCMotorSim dcMotorSim;
    private final SparkMaxSim sparkMaxSim;
    private final DIOSim extendedLimitSim;
    private final DIOSim retractedLimitSim;

    /**
     * 
     */
    public ArmModuleIOSim() {
        this.armDcMotor = DCMotor.getNEO(1);
        this.dcMotorSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(armDcMotor, 0.01, 16.0), armDcMotor);
        this.sparkMaxSim = new SparkMaxSim(this.armMotor, this.armDcMotor);

        this.extendedLimitSim = new DIOSim(extendedLimit);
        this.retractedLimitSim = new DIOSim(retractedLimit);
    }

    @Override
    public void updateInputs(ArmModuleIOInputs inputs) {
        if (DriverStation.isDisabled()) {
            stop();
        }

        double appliedVoltage = sparkMaxSim.getAppliedOutput() * RobotController.getBatteryVoltage();
        dcMotorSim.setInput(appliedVoltage);
        dcMotorSim.update(0.02);

        double velocityRPM = Units.radiansPerSecondToRotationsPerMinute(dcMotorSim.getAngularVelocityRadPerSec());
        sparkMaxSim.iterate(velocityRPM, RobotController.getBatteryVoltage(), 0.02);

        sparkMaxSim.setMotorCurrent(dcMotorSim.getCurrentDrawAmps());
        sparkMaxSim.setBusVoltage(appliedVoltage);

        // real hall effect returns false when homed
        boolean isNearRetracted = Math.abs(getPosition() - ArmConstants.RETRACTED_ANGLE) < 2.0;
        retractedLimitSim.setValue(!isNearRetracted);

        // real hall effect returns false when homed
        boolean isNearExtended = Math.abs(getPosition() - ArmConstants.EXTENDED_ANGLE) < 2.0;
        extendedLimitSim.setValue(!isNearExtended);

        inputs.data = new ArmModuleIOData(
                true,
                sparkMaxSim.getPosition(),
                sparkMaxSim.getVelocity(),
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
