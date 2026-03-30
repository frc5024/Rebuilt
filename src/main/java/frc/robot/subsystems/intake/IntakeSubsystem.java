package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants.ArmConstants;
import frc.robot.Constants.IntakeConstants.RollerConstants;
import frc.robot.Constants.RobotConstants;

/**
 * 
 */
public class IntakeSubsystem extends SubsystemBase {
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
        // set advantage kit IO logging
        this.armModuleIO = armModuleIO;
        this.rollerModuleIO = rollerModuleIO;
        this.armInputs = new ArmModuleIOInputsAutoLogged();
        this.rollerInputs = new RollerModuleIOInputsAutoLogged();
        this.targetAngle = 0.0;
        this.targetRPM = 0.0;

        // set shuffleboard entries if in tuning mode
        if (RobotConstants.TUNING_MODE) {
            kArmSVAs = ArmConstants.getSVAs();
            kArmPIDs = ArmConstants.getPIDs();

            kRollerSVAs = RollerConstants.getSVAs();
            kRollerPIDs = RollerConstants.getPIDs();

            setShuffleboard();
        }
    }

    @Override
    public void periodic() {
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

        // update pid values if in tuning mode
        if (RobotConstants.TUNING_MODE) {
            armModuleIO.setFF(
                    armSEntry.getDouble(kArmSVAs[0]),
                    armVEntry.getDouble(kArmSVAs[1]),
                    armAEntry.getDouble(kArmSVAs[2]));

            armModuleIO.setPID(
                    armPEntry.getDouble(kArmPIDs[0]),
                    armIEntry.getDouble(kArmPIDs[1]),
                    armDEntry.getDouble(kArmPIDs[2]));

            armModuleIO.setAngle(armAngleEntry.getDouble(0.0));

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

            rollerModuleIO.setVelocity(rollerRpmEntry.getDouble(0.0));

        }

        Logger.recordOutput("Intake/Arm/IsExetended", armModuleIO.isExtended());
        Logger.recordOutput("Intake/Arm/IsRetracted", armModuleIO.isRetracted());
        Logger.recordOutput("Intake/Arm/CurrentAngle", armModuleIO.getPosition());
        Logger.recordOutput("Intake/Arm/SetPointAngle", armModuleIO.getGoalPosition());
        Logger.recordOutput("Intake/Roller/CurrentRPM", rollerModuleIO.getVelocity());
        Logger.recordOutput("Intake/Roller/SetPointRPM", rollerModuleIO.getGoalVelocity());
    }

    public void extendArm() {
        targetAngle = ArmConstants.EXTENDED_ANGLE;
    }

    public double getCurrentDrawAmps() {
        return armModuleIO.getCurrentDrawAmps();
    }

    /**
     * Returns the average velocity in rotations/sec
     */
    public double getFFCharacterizationVelocity() {
        return rollerModuleIO.getFFCharacterizationVelocity();
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

    /**
     * Runs the roller with the specified output.
     */
    public void runCharacterization(double output) {
        rollerModuleIO.runCharacterization(output);
    }

    public void stopRoller() {
        targetRPM = 0.0;
        rollerModuleIO.stop();
    }

    /**
     * 
     */
    private void setShuffleboard() {
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
