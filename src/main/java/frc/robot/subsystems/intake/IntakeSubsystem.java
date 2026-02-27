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

    static ShuffleboardTab armTab = Shuffleboard.getTab("Arm");
    GenericEntry armPEntry = armTab.add("Set kP", intakeConstants.kArmP).getEntry();
    GenericEntry armIEntry = armTab.add("Set kI", intakeConstants.kArmI).getEntry();
    GenericEntry armDEntry = armTab.add("Set kD", intakeConstants.kArmD).getEntry();

    GenericEntry armSEntry = armTab.add("Set kS", intakeConstants.kArmS).getEntry();
    GenericEntry armGEntry = armTab.add("Set kG", intakeConstants.kArmG).getEntry();
    GenericEntry armVEntry = armTab.add("Set kV", intakeConstants.kArmV).getEntry();
    GenericEntry armAEntry = armTab.add("Set kA", intakeConstants.kArmA).getEntry();

    static ShuffleboardTab rollerTab = Shuffleboard.getTab("Roller")
    GenericEntry rollerPEntry = rollerTab.add("Set kP", intakeConstants.kRollP).getEntry();
    GenericEntry rollerIEntry = rollerTab.add("Set kI", intakeConstants.kRollI).getEntry();
    GenericEntry rollerDEntry = rollerTab.add("Set kD", intakeConstants.kRollD).getEntry();

    GenericEntry rollerSEntry = rollerTab.add("Set kS", intakeConstants.kRollS).getEntry();
    GenericEntry rollerVEntry = rollerTab.add("Set kV", intakeConstants.kRollV).getEntry();
    GenericEntry rollerAEntry = rollerTab.add("Set kA", intakeConstants.kRollA).getEntry();

    private PIDController armPIDController;
    private SimpleMotorFeedforward armFeedforward;

    private PIDController rollerPIDController;
    private SimpleMotorFeedforward rollerFeedforward;

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

        armTab.addBoolean("Extended?", () -> intakeModuleIO.isIntakeExtended());
        armTab.addBoolean("Retracted?", () -> intakeModuleIO.isIntakeRetracted());

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

        armPIDController = new PIDController(intakeConstants.kArmP, intakeConstants.kArmI, intakeConstants.kArmD);
        armFeedforward = new SimpleMotorFeedforward(intakeConstants.kArmS, intakeConstants.kArmV,
                intakeConstants.kArmA);

        rollerPIDController = new PIDController(intakeConstants.kRollP, intakeConstants.kRollI, intakeConstants.kRollD);
        rollerFeedforward = new SimpleMotorFeedforward(intakeConstants.kRollS, intakeConstants.kRollV,
                intakeConstants.kRollA);
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

        armPIDController.setPID(armPEntry.getDouble(intakeConstants.kArmP), armIEntry.getDouble(intakeConstants.kArmI),
                armDEntry.getDouble(intakeConstants.kArmD));
        armFeedforward.setKs(armSEntry.getDouble(intakeConstants.kArmS));
        armFeedforward.setKv(armVEntry.getDouble(intakeConstants.kArmV));
        armFeedforward.setKa(armAEntry.getDouble(intakeConstants.kArmA));

        rollerPIDController.setPID(armPEntry.getDouble(intakeConstants.kRollP),
                armIEntry.getDouble(intakeConstants.kRollI), armDEntry.getDouble(intakeConstants.kRollD));
        rollerFeedforward.setKs(armSEntry.getDouble(intakeConstants.kArmS));
        rollerFeedforward.setKv(armVEntry.getDouble(intakeConstants.kArmV));
        rollerFeedforward.setKa(armAEntry.getDouble(intakeConstants.kArmA));

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
    }

    public void setArmPID(boolean state) {
        this.armPIDEnabled = state;
    }

    public void setRollerPID(boolean state) {
        this.rollerPIDEnabled = state;
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

    public void setArmSetVelocity(double speed) {
        this.armDesiredSpeed = speed;
    }

    public void setRollerSetVelocity(double speed) {
        this.rollerDesiredSpeed = speed;
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
}
