package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.intakeConstants;
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
    GenericEntry armPEntry = tab.add("Set Arm kP", intakeConstants.kArmP).getEntry();
    GenericEntry armIEntry = tab.add("Set Arm kI", intakeConstants.kArmI).getEntry();
    GenericEntry armDEntry = tab.add("Set Arm kD", intakeConstants.kArmD).getEntry();

    GenericEntry armSEntry = tab.add("Set Arm kS", intakeConstants.kArmS).getEntry();
    GenericEntry armVEntry = tab.add("Set Arm kV", intakeConstants.kArmV).getEntry();
    GenericEntry armAEntry = tab.add("Set Arm kA", intakeConstants.kArmA).getEntry();
    GenericEntry armGEntry = tab.add("Set Arm kG", intakeConstants.kArmG).getEntry();

    GenericEntry rollerPEntry = tab.add("Set Arm kP", intakeConstants.kRollP).getEntry();
    GenericEntry rollerIEntry = tab.add("Set Arm kI", intakeConstants.kRollI).getEntry();
    GenericEntry rollerDEntry = tab.add("Set Arm kD", intakeConstants.kRollD).getEntry();

    GenericEntry rollerSEntry = tab.add("Set Arm kS", intakeConstants.kRollS).getEntry();
    GenericEntry rollerVEntry = tab.add("Set Arm kV", intakeConstants.kRollV).getEntry();
    GenericEntry rollerAEntry = tab.add("Set Arm kA", intakeConstants.kRollA).getEntry();
    GenericEntry rollerGEntry = tab.add("Set Arm kG", intakeConstants.kRollG).getEntry();

    private PIDController armPIDController;
    private ArmFeedforward armFeedforward;

    private PIDController rollerPIDController;
    private SimpleMotorFeedforward rollerFeedforward;

    public boolean armPIDEnabled;
    public boolean rollerPIDEnabled;

    /**
     * 
     */
    public IntakeSubsystem(IntakeModuleIO intakeModuleIO) {
        this.intakeModuleIO = intakeModuleIO;
        this.inputs = new IntakeModuleIOInputsAutoLogged();

        tab.addBoolean("Extended?", () -> intakeModuleIO.isIntakeExtended());
        tab.addBoolean("Retracted?", () -> intakeModuleIO.isIntakeRetracted());

        armPEntry.setDouble(intakeConstants.kArmP);
        armIEntry.setDouble(intakeConstants.kArmI);
        armDEntry.setDouble(intakeConstants.kArmD);

        armSEntry.setDouble(intakeConstants.kArmS);
        armVEntry.setDouble(intakeConstants.kArmV);
        armGEntry.setDouble(intakeConstants.kArmG);
        armAEntry.setDouble(intakeConstants.kArmA);

        rollerPEntry.setDouble(intakeConstants.kRollP);
        rollerIEntry.setDouble(intakeConstants.kRollI);
        rollerDEntry.setDouble(intakeConstants.kRollD);

        rollerSEntry.setDouble(intakeConstants.kRollS);
        rollerVEntry.setDouble(intakeConstants.kRollV);
        rollerGEntry.setDouble(intakeConstants.kRollG);
        rollerAEntry.setDouble(intakeConstants.kRollA);
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
