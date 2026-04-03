package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.statemachine.StateMachineSubsystem;
import frc.robot.Constants.IntakeConstants.ArmConstants;
import frc.robot.Constants.IntakeConstants.RollerConstants;

/**
 * 
 */
public class IntakeSubsystem extends StateMachineSubsystem {
    // Advantagekit logging
    private final ArmModuleIO armModuleIO;
    private final RollerModuleIO rollerModuleIO;
    protected final ArmModuleIOInputsAutoLogged armInputs;
    protected final RollerModuleIOInputsAutoLogged rollerInputs;

    // Variables
    private double targetAngle;
    private double targetRPM;

    private double[] kArmSVAs;
    private double[] kArmPIDs;

    private double[] kRollerSVAs;
    private double[] kRollerPIDs;

    // Shuffleboard entries
    private ShuffleboardTab armTab;

    private GenericEntry armSEntry;
    private GenericEntry armVEntry;
    private GenericEntry armAEntry;

    private GenericEntry armPEntry;
    private GenericEntry armIEntry;
    private GenericEntry armDEntry;

    private GenericEntry armAngleEntry;
    private GenericEntry armExtendEntry;

    private ShuffleboardTab rollerTab;

    private GenericEntry rollerSEntry;
    private GenericEntry rollerVEntry;
    private GenericEntry rollerAEntry;

    private GenericEntry rollerPEntry;
    private GenericEntry rollerIEntry;
    private GenericEntry rollerDEntry;

    private GenericEntry rollerRpmEntry;

    /**
     * 
     */
    public IntakeSubsystem(ArmModuleIO armModuleIO, RollerModuleIO rollerModuleIO) {
        super("Intake");

        // set advantage kit IO logging
        this.armModuleIO = armModuleIO;
        this.rollerModuleIO = rollerModuleIO;
        this.armInputs = new ArmModuleIOInputsAutoLogged();
        this.rollerInputs = new RollerModuleIOInputsAutoLogged();
        this.targetAngle = 0.0;
        this.targetRPM = 0.0;
    }

    @Override
    public void periodic() {
        super.periodic();

        // process hardware inputs
        armModuleIO.updateInputs(armInputs);
        rollerModuleIO.updateInputs(rollerInputs);
        Logger.processInputs("Intake/Arm", armInputs);
        Logger.processInputs("Intake/Roller", rollerInputs);

        // update the arm voltage from pid controller & feedforward
        armModuleIO.setAngle(targetAngle);
        rollerModuleIO.setVelocity(targetRPM);

        // reset the relative encoder if retracted
        if (armModuleIO.isRetracted()) {
            armModuleIO.setPosition(0.0);
        }

        Logger.recordOutput("Subsystems/Intake/Arm/IsExetended", armModuleIO.isExtended());
        Logger.recordOutput("Subsystems/Intake/Arm/IsRetracted", armModuleIO.isRetracted());
        Logger.recordOutput("Subsystems/Intake/Arm/CurrentAngle", armModuleIO.getPosition());
        Logger.recordOutput("Subsystems/Intake/Arm/TargetAngle", armModuleIO.getGoalPosition());
        Logger.recordOutput("Subsystems/Intake/Roller/CurrentRPM", rollerModuleIO.getVelocity());
        Logger.recordOutput("Subsystems/Intake/Roller/TargetRPM", rollerModuleIO.getGoalVelocity());
    }

    public void extendArm() {
        targetAngle = ArmConstants.EXTENDED_ANGLE;
    }

    public double getCurrentDrawAmps() {
        return armModuleIO.getCurrentDrawAmps();
    }

    public double getPosition() {
        return armModuleIO.getPosition();
    }

    public void intakeRoller() {
        setVelocity(RollerConstants.INTAKE_RPM);
    }

    public void outtakeRoller() {
        setVelocity(RollerConstants.OUTTAKE_RPM);
    }

    public void retractArm() {
        setAngle(ArmConstants.RETRACTED_ANGLE);
    }

    public boolean isRetracted() {
        return armModuleIO.isRetracted();
    }

    public boolean isExtended() {
        return armModuleIO.isExtended();
    }

    public void setAngle(double targetAngle) {
        this.targetAngle = targetAngle;
    }

    public void setVelocity(double targetRPM) {
        this.targetRPM = targetRPM;
    }

    public void stopRoller() {
        targetRPM = 0.0;
        rollerModuleIO.stop();
    }

    /**
     * 
     */
    @Override
    protected void setShuffleboard() {
        this.kArmSVAs = ArmConstants.getSVAs();
        this.kArmPIDs = ArmConstants.getPIDs();

        this.kRollerSVAs = RollerConstants.getSVAs();
        this.kRollerPIDs = RollerConstants.getPIDs();
    }

    @Override
    protected void setShuffleboardTab() {
        armTab = Shuffleboard.getTab("Intake/Arm");
        armSEntry = armTab.add("SET S", kArmSVAs[0]).getEntry();
        armVEntry = armTab.add("SET V", kArmSVAs[1]).getEntry();
        armAEntry = armTab.add("SET A", kArmSVAs[2]).getEntry();

        armSEntry.setDouble(kArmSVAs[0]);
        armVEntry.setDouble(kArmSVAs[1]);
        armAEntry.setDouble(kArmSVAs[2]);

        armPEntry = armTab.add("SET P", kArmPIDs[0]).getEntry();
        armIEntry = armTab.add("SET I", kArmPIDs[1]).getEntry();
        armDEntry = armTab.add("SET D", kArmPIDs[2]).getEntry();

        armPEntry.setDouble(kArmPIDs[0]);
        armIEntry.setDouble(kArmPIDs[1]);
        armDEntry.setDouble(kArmPIDs[2]);

        armAngleEntry = armTab.add("SET ANGLE", 0.0).getEntry();
        armAngleEntry.setDouble(0.0);

        armExtendEntry = armTab.add("EXTEND", false)
                .withWidget("Toggle Button")
                .getEntry();

        rollerTab = Shuffleboard.getTab("Intake/Roller");
        rollerSEntry = rollerTab.add("SET S", kRollerSVAs[0]).getEntry();
        rollerVEntry = rollerTab.add("SET V", kRollerSVAs[1]).getEntry();
        rollerAEntry = rollerTab.add("SET A", kRollerSVAs[2]).getEntry();

        rollerSEntry.setDouble(kRollerSVAs[0]);
        rollerVEntry.setDouble(kRollerSVAs[1]);
        rollerAEntry.setDouble(kRollerSVAs[2]);

        rollerPEntry = rollerTab.add("SET P", kRollerPIDs[0]).getEntry();
        rollerIEntry = rollerTab.add("SET I", kRollerPIDs[1]).getEntry();
        rollerDEntry = rollerTab.add("SET D", kRollerPIDs[2]).getEntry();

        rollerPEntry.setDouble(kRollerPIDs[0]);
        rollerIEntry.setDouble(kRollerPIDs[1]);
        rollerDEntry.setDouble(kRollerPIDs[2]);

        rollerRpmEntry = rollerTab.add("SET RPM", 0.0).getEntry();
        rollerRpmEntry.setDouble(0.0);
    }

    @Override
    protected void setShuffleboardEntries() {
        armModuleIO.setFF(
                armSEntry.getDouble(kArmSVAs[0]),
                armVEntry.getDouble(kArmSVAs[1]),
                armAEntry.getDouble(kArmSVAs[2]));

        armModuleIO.setPID(
                armPEntry.getDouble(kArmPIDs[0]),
                armIEntry.getDouble(kArmPIDs[1]),
                armDEntry.getDouble(kArmPIDs[2]));

        setAngle(armAngleEntry.getDouble(0.0));

        if (armExtendEntry.getBoolean(false)) {
            armModuleIO.setAngle(ArmConstants.EXTENDED_ANGLE);
        }

        rollerModuleIO.setFF(
                rollerSEntry.getDouble(kRollerSVAs[0]),
                rollerVEntry.getDouble(kRollerSVAs[1]),
                rollerAEntry.getDouble(kRollerSVAs[2]));

        rollerModuleIO.setPID(
                rollerPEntry.getDouble(kRollerPIDs[0]),
                rollerIEntry.getDouble(kRollerPIDs[1]),
                rollerDEntry.getDouble(kRollerPIDs[2]));

        setVelocity(rollerRpmEntry.getDouble(0.0));
    }

    /**
     * Overrides for SysId routines
     */
    @Override
    public double getFFCharacterizationVelocity() {
        return rollerModuleIO.getFFCharacterizationVelocity();
    }

    @Override
    public void runCharacterization(double voltage) {
        rollerModuleIO.runCharacterization(voltage);
    }

    /**
     * Commands
     */
    public Command IntakeCommand() {
        return Commands.runOnce(() -> intakeRoller(), this);
    }

    public Command OuttakeCommand() {
        return Commands.runOnce(() -> outtakeRoller(), this);
    }

    public Command ExtendArmCommand() {
        return Commands.sequence(
                Commands.runOnce(() -> intakeRoller(), this),
                Commands.runOnce(() -> extendArm(), this));
    }

    public Command RetractArmCommand() {
        return Commands.sequence(
                Commands.runOnce(() -> rollerModuleIO.stop(), this),
                Commands.runOnce(() -> retractArm(), this));
    }
}
