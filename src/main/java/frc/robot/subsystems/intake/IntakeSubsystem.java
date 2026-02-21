package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.Intake.IntakeExtendArm;
import frc.robot.commands.Intake.IntakeRetractArm;
import frc.robot.commands.Intake.IntakeSpinMotor;
import frc.robot.commands.Intake.OuttakeSpinMotor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

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

        tab.addBoolean("Extended?",() -> intakeModuleIO.isIntakeExtended());
        tab.addBoolean("Retracted?",() -> intakeModuleIO.isIntakeRetracted());
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        intakeModuleIO.updateInputs(inputs);
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
