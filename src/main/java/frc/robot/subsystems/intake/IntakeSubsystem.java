package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.Intake.IntakeExtendArm;
import frc.robot.commands.Intake.IntakeRetractArm;
import frc.robot.commands.Intake.IntakeSpinMotor;
import frc.robot.commands.Intake.OuttakeSpinMotor;

/**
 * 
 */
public class IntakeSubsystem extends SubsystemBase {
    private final IntakeModuleIO intakeModuleIO;
    protected final IntakeModuleIOInputsAutoLogged inputs;
    static ShuffleboardTab tab = Shuffleboard.getTab("intakeMotor");

    /**
     * 
     */
    public IntakeSubsystem(IntakeModuleIO intakeModuleIO) {
        this.intakeModuleIO = intakeModuleIO;
        this.inputs = new IntakeModuleIOInputsAutoLogged();

        tab.addBoolean("Extended?", () -> intakeModuleIO.isIntakeExtended());
        tab.addBoolean("Retracted?", () -> intakeModuleIO.isIntakeRetracted());
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        intakeModuleIO.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);

        Logger.recordOutput("Intake/IsExetended", intakeModuleIO.isIntakeExtended());
        Logger.recordOutput("Intake/IsRetracted", intakeModuleIO.isIntakeRetracted());
        Logger.recordOutput("Intake/IsIntaking", intakeModuleIO.isIntakeIntaking());
        Logger.recordOutput("Intake/ArmAngle", Units.radiansToDegrees(intakeModuleIO.getPosition()));
    }

    public double getCurrentDrawAmps() {
        return intakeModuleIO.getCurrentDrawAmps();
    }

    public double getPosition() {
        return intakeModuleIO.getPosition();
    }

    public void setIntakeSpeed(double speed) {
        intakeModuleIO.setIntake(speed);
    }

    public void setArmSpeed(double speed) {
        intakeModuleIO.setArm(speed);
    }

    public Command IntakeSpin() {
        return new IntakeSpinMotor(this);
    }

    public Command OuttakeSpin() {
        return new OuttakeSpinMotor(this);
    }

    public Command ExtendSpin() {
        return new IntakeExtendArm(this);
    }

    public Command RetractSpin() {
        return new IntakeRetractArm(this);
    }

    public boolean isIntakeRetracted() {
        return intakeModuleIO.isIntakeRetracted();
    }

    public boolean isIntakeExtended() {
        return intakeModuleIO.isIntakeExtended();
    }
}
