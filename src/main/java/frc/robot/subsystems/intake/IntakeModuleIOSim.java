package frc.robot.subsystems.intake;

import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.sim.SparkMaxSim;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/**
 * 
 */
public class IntakeModuleIOSim extends IntakeModuleIOSparkMaxFlex {
    // Hardware objects
    private final DCMotor armDcMotor;
    private final DCMotorSim armMotorSim;
    private final SparkMaxSim armSparkMaxSim;
    private final DCMotor intakeDcMotor;
    private final DCMotorSim intakeMotorSim;
    private final SparkFlexSim intakeSparkFlexSim;

    private double armVoltageRequest;
    private double intakeVoltageRequest;

    /**
     * 
     */
    public IntakeModuleIOSim() {
        this.armDcMotor = DCMotor.getNEO(1);
        this.armMotorSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(armDcMotor, 0.01, 16.0), armDcMotor);
        this.armSparkMaxSim = new SparkMaxSim(this.armMotor, this.armDcMotor);

        this.intakeDcMotor = DCMotor.getNEO(1);
        this.intakeMotorSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(intakeDcMotor, 0.01, 4.0),
                intakeDcMotor);
        this.intakeSparkFlexSim = new SparkFlexSim(this.intakeMotor, this.intakeDcMotor);

        this.armVoltageRequest = 0.0;
        this.intakeVoltageRequest = 0.0;
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

        intakeSparkFlexSim.setPosition(intakeMotorSim.getAngularPositionRotations());
        intakeSparkFlexSim.setVelocity(intakeMotorSim.getAngularVelocityRPM());

        inputs.data = new IntakeModuleIOData(
                true,
                armSparkMaxSim.getPosition(),
                armSparkMaxSim.getVelocity(),
                armVoltageRequest,
                0.0,
                armSparkMaxSim.getMotorCurrent(),
                0.0,
                true,
                intakeSparkFlexSim.getPosition(),
                intakeSparkFlexSim.getVelocity(),
                intakeVoltageRequest,
                0.0,
                intakeSparkFlexSim.getMotorCurrent(),
                0.0);
    }

    @Override
    public double getCurrentDrawAmps() {
        return armSparkMaxSim.getMotorCurrent() + intakeSparkFlexSim.getMotorCurrent();
    }

    @Override
    public double getArmPosition() {
        return armSparkMaxSim.getPosition();
    }

    @Override
    public boolean isIntakeExtended() {
        return Units.radiansToDegrees(armSparkMaxSim.getPosition()) >= 20.0;
    }

    @Override
    public boolean isIntakeIntaking() {
        return intakeMotorSim.getAngularVelocityRPM() > 0.0;
    }

    @Override
    public boolean isIntakeRetracted() {
        return Units.radiansToDegrees(armSparkMaxSim.getPosition()) <= -100.0;
    }

    @Override
    public void setArm(double voltage) {
        armVoltageRequest = MathUtil.clamp(-voltage * 12, -12.0, 12.0);
        armMotorSim.setInputVoltage(armVoltageRequest);
    }

    @Override
    public void setIntake(double voltage) {
        intakeVoltageRequest = MathUtil.clamp(voltage * 12, -12.0, 12.0);
        intakeMotorSim.setInputVoltage(intakeVoltageRequest);
    }
}
