package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
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
import frc.robot.commands.Intake.PIDIntakeSpin;
import frc.robot.commands.Intake.PIDOuttakeSpin;
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
    private double armCurrentAngle;
    private double armDesiredAngle;
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

        // set arm motor voltage by PID
        if (armPIDEnabled) {
            armPIDCalculate();
        }

        // set intkake motor voltage by PID
        if (rollerPIDEnabled) {
            rollerPIDCalculate();
        }

        // update pid values if in tuning mode
        if (RobotConstants.TUNING_MODE) {
            armPIDController.setPID(armPEntry.getDouble(intakeConstants.kArmP),
                    armIEntry.getDouble(intakeConstants.kArmI),
                    armDEntry.getDouble(intakeConstants.kArmD));
            armFeedforward.setKs(armSEntry.getDouble(intakeConstants.kArmS));
            armFeedforward.setKv(armVEntry.getDouble(intakeConstants.kArmV));
            armFeedforward.setKa(armAEntry.getDouble(intakeConstants.kArmA));

            rollerPIDController.setPID(rollerPEntry.getDouble(intakeConstants.kRollP),
                    rollerIEntry.getDouble(intakeConstants.kRollI), rollerDEntry.getDouble(intakeConstants.kRollD));
            rollerFeedforward.setKs(rollerSEntry.getDouble(intakeConstants.kRollS));
            rollerFeedforward.setKv(rollerVEntry.getDouble(intakeConstants.kRollV));
            rollerFeedforward.setKa(rollerAEntry.getDouble(intakeConstants.kRollA));
        }

        Logger.recordOutput("Intake/Arm/IsExetended", intakeModuleIO.isIntakeExtended());
        Logger.recordOutput("Intake/Arm/IsRetracted", intakeModuleIO.isIntakeRetracted());
        Logger.recordOutput("Intake/Arm/IsIntaking", intakeModuleIO.isIntakeIntaking());
        Logger.recordOutput("Intake/Arm/Angle", Units.radiansToDegrees(intakeModuleIO.getArmPosition()));
        Logger.recordOutput("Intake/Roller/CurrentRPM", intakeModuleIO.getIntakeVelocity());
    }

    public double getCurrentDrawAmps() {
        return intakeModuleIO.getCurrentDrawAmps();
    }

    public double getArmPosition() {
        return intakeModuleIO.getArmPosition();
    }

    public void setArmPID(boolean armPIDEnabled) {
        this.armPIDEnabled = armPIDEnabled;

        // turn off arm motor if not using PID
        if (!armPIDEnabled) {
            intakeModuleIO.setArm(0);
        }
    }

    public void setRollerPID(boolean rollerPIDEnabled) {
        this.rollerPIDEnabled = rollerPIDEnabled;

        // turn off roller motor if not using PID
        if (!rollerPIDEnabled) {
            intakeModuleIO.setIntake(0);
        }
    }

    public void setRollerSpeed(double speed) {
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

    public boolean isIntakeRetracted() {
        return intakeModuleIO.isIntakeRetracted();
    }

    public boolean isIntakeExtended() {
        return intakeModuleIO.isIntakeExtended();
    }

    public void setRollerDesiredSpeed(double desiredSpeed) {
        this.rollerDesiredSpeed = desiredSpeed;
        this.rollerPIDController.reset();
        intakeModuleIO.setIntakeEncoderPosition(0.0);
    }

    public void armPIDCalculate() {
        armCurrentSpeed = intakeModuleIO.getArmVelocity();

        double armPIDoutput = armPIDController.calculate(armCurrentSpeed, armDesiredSpeed);
        double armFeedForwardOutput = armFeedforward.calculate(armCurrentSpeed);
        double totalOutput = armPIDoutput + armFeedForwardOutput;
        double voltageRequest = MathUtil.clamp(totalOutput, -12.0, 12.0);

        setArmSpeed(voltageRequest);
    }

    public void rollerPIDCalculate() {
        this.rollerCurrentSpeed = intakeModuleIO.getIntakeVelocity();

        double rollerPIDoutput = rollerPIDController.calculate(rollerCurrentSpeed, rollerDesiredSpeed);
        double rollerFeedforwardOutput = rollerFeedforward.calculate(rollerDesiredSpeed);
        double totalOutput = rollerPIDoutput + rollerFeedforwardOutput;
        double voltageRequest = MathUtil.clamp(totalOutput, -12.0, 12.0);

        setRollerSpeed(voltageRequest);
    }

    /**
     * 
     */
    private void setShuffleboard() {
        armTab = Shuffleboard.getTab("Arm");
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

    /**
     * Commands
     */

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

    public Command retractPIDCommand() {
        return new PIDRetractArm(this);
    }

    public Command extendPIDCommand() {
        return new PIDExtendArm(this);
    }

    public Command intakePIDCommand() {
        return new PIDIntakeSpin(this);
    }

    public Command outtakePIDCommand() {
        return new PIDOuttakeSpin(this);
    }
}
