package frc.robot.subsystems.intake;

import com.revrobotics.sim.SparkMaxSim;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/**
 * 
 */
public class IntakeModuleIOSim extends IntakeModuleIOSparkMax {
    // Hardware objects
    private final DCMotor dcMotor;
    private final SparkMaxSim armSparkMaxSim;
    private final DCMotorSim armMotorSim;
    private final DCMotorSim intakeMotorSim;
    private final SparkMaxSim intakeSparkMaxSim;

    private double armVoltageRequest;
    private double intakeVoltageRequest;

    /**
     * 
     */
    public IntakeModuleIOSim() {
        this.dcMotor = DCMotor.getNEO(1);
        this.armSparkMaxSim = new SparkMaxSim(this.armMotor, this.dcMotor);
        this.armMotorSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(dcMotor, 0.01, 4.0), dcMotor);
        this.intakeMotorSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(dcMotor, 0.01, 4.0), dcMotor);
        this.intakeSparkMaxSim = new SparkMaxSim(this.intakeMotor, this.dcMotor);

        this.armVoltageRequest = 0.0;
        this.intakeVoltageRequest = 0.0;
    }

    @Override
    public double getCurrentDrawAmps() {
        return 0.0;
    }

    @Override
    public double getPosition() {
        return armSparkMaxSim.getPosition();
    }

    @Override
    public void setArm(double voltage) {
        armVoltageRequest = MathUtil.clamp(voltage * 12, -12.0, 12.0);
        armMotorSim.setInputVoltage(armVoltageRequest);
    }

    @Override
    public void setIntake(double voltage) {
        intakeVoltageRequest = MathUtil.clamp(voltage * 12, -12.0, 12.0);
        intakeMotorSim.setInputVoltage(intakeVoltageRequest);
    }

    @Override
    public void updateInputs(IntakeModuleIOInputs inputs) {
        if (DriverStation.isDisabled()) {
            stop();
        }

        armMotorSim.update(0.02);
        intakeMotorSim.update(0.02);

        armSparkMaxSim.setPosition(armMotorSim.getAngularPositionRotations());
        armSparkMaxSim.setVelocity(armMotorSim.getAngularVelocityRPM());

        intakeSparkMaxSim.setPosition(intakeMotorSim.getAngularPositionRotations());
        intakeSparkMaxSim.setVelocity(intakeMotorSim.getAngularVelocityRPM());

        inputs.data = new IntakeModuleIOData(
                true,
                armSparkMaxSim.getPosition(),
                armSparkMaxSim.getVelocity(),
                armVoltageRequest,
                0.0,
                armSparkMaxSim.getMotorCurrent(),
                0.0,
                true,
                intakeSparkMaxSim.getPosition(),
                intakeSparkMaxSim.getVelocity(),
                intakeVoltageRequest,
                0.0,
                intakeSparkMaxSim.getMotorCurrent(),
                0.0);
    }
}
