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

    private double[] kRollerSVAs;
    private double[] kRollerPIDs;

    // Shuffleboard entries
    private GenericEntry armExtendEntry;

    private ShuffleboardTab rollerTab;

    private GenericEntry rollerSEntry;
    private GenericEntry rollerVEntry;
    private GenericEntry rollerAEntry;

    private GenericEntry rollerPEntry;
    private GenericEntry rollerIEntry;
    private GenericEntry rollerDEntry;

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
        targetRPM = RollerConstants.INTAKE_RPM;
    }

    public void outtakeRoller() {
        targetRPM = RollerConstants.OUTTAKE_RPM;
    }

    public void retractArm() {
        targetAngle = ArmConstants.RETRACTED_ANGLE;
    }

    public boolean isRetracted() {
        return armModuleIO.isRetracted();
    }

    public boolean isExtended() {
        return armModuleIO.isExtended();
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
        this.kSVAs = ArmConstants.getSVAs();
        this.kPIDs = ArmConstants.getPIDs();

        this.kRollerSVAs = RollerConstants.getSVAs();
        this.kRollerPIDs = RollerConstants.getPIDs();
    }

    @Override
    protected void setShuffleboardTab() {
        super.setShuffleboardTab();

        armExtendEntry = tab.add("EXTEND", false)
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
    }

    @Override
    protected void setShuffleboardEntries() {
        armModuleIO.setFF(
                sEntry.getDouble(kSVAs[0]),
                vEntry.getDouble(kSVAs[1]),
                aEntry.getDouble(kSVAs[2]));

        armModuleIO.setPID(
                pEntry.getDouble(kPIDs[0]),
                iEntry.getDouble(kPIDs[1]),
                dEntry.getDouble(kPIDs[2]));

        armModuleIO.setAngle(angleEntry.getDouble(0.0));

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

        rollerModuleIO.setVelocity(rpmEntry.getDouble(0.0));
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
