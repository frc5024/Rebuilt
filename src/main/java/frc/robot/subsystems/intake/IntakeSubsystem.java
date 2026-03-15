package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.intakeConstants;
import frc.robot.commands.Intake.IntakeExtendArm;
import frc.robot.commands.Intake.IntakeRetractArm;
import frc.robot.commands.Intake.IntakeSpinMotor;
import frc.robot.commands.Intake.OuttakeSpinMotor;
import frc.robot.commands.Intake.PIDExtendArm;
import frc.robot.commands.Intake.PIDRetractArm;

/**
 * 
 */
public class IntakeSubsystem extends SubsystemBase {
    // Advantagekit logging
    private final IntakeModuleIO intakeModuleIO;
    protected final IntakeModuleIOInputsAutoLogged inputs;

    // PID
    private PIDController armPIDController;
    private PIDController rollerPIDController;

    private SimpleMotorFeedforward armFeedforward;
    private SimpleMotorFeedforward rollerFeedforward;

    // Shuffleboard entries
    private ShuffleboardTab armTab;
    private GenericEntry armPEntry;
    private GenericEntry armIEntry;
    private GenericEntry armDEntry;

    private GenericEntry armSEntry;
    private GenericEntry armGEntry;
    private GenericEntry armVEntry;
    private GenericEntry armAEntry;

    private ShuffleboardTab rollerTab;
    private GenericEntry rollerPEntry;
    private GenericEntry rollerIEntry;
    private GenericEntry rollerDEntry;

    private GenericEntry rollerSEntry;
    private GenericEntry rollerVEntry;
    private GenericEntry rollerAEntry;

    // Variables
    public boolean armPIDEnabled;
    public boolean rollerPIDEnabled;

    private double armCurrentSpeed;
    private double armDesiredSpeed;
    private double rollerCurrentSpeed;
    private double rollerDesiredSpeed;

    /**
     * 
     */
    public IntakeSubsystem(IntakeModuleIO intakeModuleIO) {
        this.intakeModuleIO = intakeModuleIO;
        this.inputs = new IntakeModuleIOInputsAutoLogged();

        armPIDController = new PIDController(intakeConstants.kArmP, intakeConstants.kArmI, intakeConstants.kArmD);
        armFeedforward = new SimpleMotorFeedforward(intakeConstants.kArmS, intakeConstants.kArmV,
                intakeConstants.kArmA);

        rollerPIDController = new PIDController(intakeConstants.kRollP, intakeConstants.kRollI, intakeConstants.kRollD);
        rollerFeedforward = new SimpleMotorFeedforward(intakeConstants.kRollS, intakeConstants.kRollV,
                intakeConstants.kRollA);

        // set shuffleboard entries if in tuning mode
        if (RobotConstants.TUNING_MODE) {
            setShuffleboard();
        }
    }

    @Override
    public void periodic() {
        // process hardware inputs
        intakeModuleIO.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);

        // update pid values if in tuning mode
        if (RobotConstants.TUNING_MODE) {
            armPIDController.setPID(armPEntry.getDouble(intakeConstants.kArmP),
                    armIEntry.getDouble(intakeConstants.kArmI),
                    armDEntry.getDouble(intakeConstants.kArmD));
            armFeedforward.setKs(armSEntry.getDouble(intakeConstants.kArmS));
            armFeedforward.setKv(armVEntry.getDouble(intakeConstants.kArmV));
            armFeedforward.setKa(armAEntry.getDouble(intakeConstants.kArmA));

            rollerPIDController.setPID(armPEntry.getDouble(intakeConstants.kRollP),
                    armIEntry.getDouble(intakeConstants.kRollI), armDEntry.getDouble(intakeConstants.kRollD));
            rollerFeedforward.setKs(armSEntry.getDouble(intakeConstants.kArmS));
            rollerFeedforward.setKv(armVEntry.getDouble(intakeConstants.kArmV));
            rollerFeedforward.setKa(armAEntry.getDouble(intakeConstants.kArmA));
        }

        if (armPIDEnabled) {
            armPIDCalculate();
        } else {
            intakeModuleIO.setArm(0);
        }

        if (rollerPIDEnabled) {
            rollerPIDCalculate();
        } else {
            intakeModuleIO.setIntake(0);
        }

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

    public void setArmPID(boolean armPIDEnabled) {
        this.armPIDEnabled = armPIDEnabled;
    }

    public void setRollerPID(boolean rollerPIDEnabled) {
        this.rollerPIDEnabled = rollerPIDEnabled;
    }

    public void setIntakeSpeed(double speed) {
        intakeModuleIO.setIntake(speed);
    }

    public void setArmSpeed(double speed) {
        intakeModuleIO.setArm(speed);
    }

    public void setArmSetVelocity(double speed) {
        this.armDesiredSpeed = speed;
    }

    public void setRollerSetVelocity(double speed) {
        this.rollerDesiredSpeed = speed;
    }

    public Command IntakeCommand() {
        return new IntakeSpinMotor(this);
    }

    public Command OuttakeCommand() {
        return new OuttakeSpinMotor(this);
    }

    public Command ExtendArmCommand() {
        return new IntakeExtendArm(this);
    }

    public Command RetractArmCommand() {
        return new IntakeRetractArm(this);
    }

    public Command retractArmPIDCommand() {
        return new PIDRetractArm(this);
    }

    public Command extendArmPIDCommand() {
        return new PIDExtendArm(this);
    }

    public boolean isIntakeRetracted() {
        return intakeModuleIO.isIntakeRetracted();
    }

    public boolean isIntakeExtended() {
        return intakeModuleIO.isIntakeExtended();
    }

    public void armPIDCalculate() {
        armCurrentSpeed = intakeModuleIO.getArmVelocity();

        double armPIDoutput = armPIDController.calculate(armCurrentSpeed, armDesiredSpeed);
        double armFeedForwardOutput = armFeedforward.calculate(armPIDoutput);
        double totalOutput = armPIDoutput + armFeedForwardOutput;

        setArmSpeed(totalOutput);
    }

    public void rollerPIDCalculate() {
        rollerCurrentSpeed = intakeModuleIO.getIntakeVelocity();

        double rollerPIDoutput = rollerPIDController.calculate(rollerCurrentSpeed, rollerDesiredSpeed);
        double rollerFeedforwardOutput = rollerFeedforward.calculate(rollerPIDoutput);
        double totalOutput = rollerPIDoutput + rollerFeedforwardOutput;

        setIntakeSpeed(totalOutput);
    }

    /**
     * 
     */
    private void setShuffleboard() {
        armTab = Shuffleboard.getTab("Arm");

        armPEntry.setDouble(intakeConstants.kArmP);
        armIEntry.setDouble(intakeConstants.kArmI);
        armDEntry.setDouble(intakeConstants.kArmD);

        armSEntry.setDouble(intakeConstants.kArmS);
        armVEntry.setDouble(intakeConstants.kArmV);
        armAEntry.setDouble(intakeConstants.kArmA);

        rollerPEntry.setDouble(intakeConstants.kRollP);
        rollerIEntry.setDouble(intakeConstants.kRollI);
        rollerDEntry.setDouble(intakeConstants.kRollD);

        rollerSEntry.setDouble(intakeConstants.kRollS);
        rollerVEntry.setDouble(intakeConstants.kRollV);
        rollerAEntry.setDouble(intakeConstants.kRollA);

        armPEntry = armTab.add("Set kP", intakeConstants.kArmP).getEntry();
        armIEntry = armTab.add("Set kI", intakeConstants.kArmI).getEntry();
        armDEntry = armTab.add("Set kD", intakeConstants.kArmD).getEntry();

        armSEntry = armTab.add("Set kS", intakeConstants.kArmS).getEntry();
        armGEntry = armTab.add("Set kG", intakeConstants.kArmG).getEntry();
        armVEntry = armTab.add("Set kV", intakeConstants.kArmV).getEntry();
        armAEntry = armTab.add("Set kA", intakeConstants.kArmA).getEntry();

        rollerTab = Shuffleboard.getTab("Roller");
        rollerPEntry = rollerTab.add("Set kP", intakeConstants.kRollP).getEntry();
        rollerIEntry = rollerTab.add("Set kI", intakeConstants.kRollI).getEntry();
        rollerDEntry = rollerTab.add("Set kD", intakeConstants.kRollD).getEntry();

        rollerSEntry = rollerTab.add("Set kS", intakeConstants.kRollS).getEntry();
        rollerVEntry = rollerTab.add("Set kV", intakeConstants.kRollV).getEntry();
        rollerAEntry = rollerTab.add("Set kA", intakeConstants.kRollA).getEntry();

        armTab.addBoolean("Extended?", () -> intakeModuleIO.isIntakeExtended());
        armTab.addBoolean("Retracted?", () -> intakeModuleIO.isIntakeRetracted());
    }
}
