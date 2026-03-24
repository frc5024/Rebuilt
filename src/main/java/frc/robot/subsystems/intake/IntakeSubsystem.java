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

    // Shuffleboard entries
    private ShuffleboardTab armTab;

    private GenericEntry armPEntry;
    private GenericEntry armIEntry;
    private GenericEntry armDEntry;

    private GenericEntry armSEntry;
    private GenericEntry armVEntry;
    private GenericEntry armAEntry;

    private GenericEntry armExtendEntry;
    private GenericEntry armRetractEntry;

    private ShuffleboardTab rollerTab;

    private GenericEntry rollerPEntry;
    private GenericEntry rollerIEntry;
    private GenericEntry rollerDEntry;

    private GenericEntry rollerSEntry;
    private GenericEntry rollerVEntry;
    private GenericEntry rollerAEntry;

    private GenericEntry rollerIntakeEntry;
    private GenericEntry rollerOuttakeEntry;
    private GenericEntry rollerStopEntry;

    // Variables
    public boolean armPIDEnabled;
    public boolean rollerPIDEnabled;

    private double[] kArmPIDs;
    private double[] kArmSVAs;
    private double[] kRollerPIDs;
    private double[] kRollerSVAs;

    /**
     * 
     */
    public IntakeSubsystem(ArmModuleIO armModuleIO, RollerModuleIO rollerModuleIO) {
        this.armModuleIO = armModuleIO;
        this.rollerModuleIO = rollerModuleIO;
        this.armInputs = new ArmModuleIOInputsAutoLogged();
        this.rollerInputs = new RollerModuleIOInputsAutoLogged();

        // set shuffleboard entries if in tuning mode
        if (RobotConstants.TUNING_MODE) {
            kArmPIDs = ArmConstants.getPIDs();
            kArmSVAs = ArmConstants.getSVAs();
            kRollerPIDs = RollerConstants.getPIDs();
            kRollerSVAs = RollerConstants.getSVAs();

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
        armModuleIO.setVoltage();

        // reset the relative encoder if retracted
        if (armModuleIO.isRetracted()) {
            armModuleIO.setPosition(0.0);
        }

        // update pid values if in tuning mode
        if (RobotConstants.TUNING_MODE) {
            armModuleIO.setPID(armPEntry.getDouble(kArmPIDs[0]), armIEntry.getDouble(kArmPIDs[1]),
                    armDEntry.getDouble(kArmPIDs[2]));
            armModuleIO.setFF(armSEntry.getDouble(kArmSVAs[0]), armVEntry.getDouble(kArmSVAs[1]),
                    armAEntry.getDouble(kArmSVAs[2]));

            rollerModuleIO.setPID(rollerPEntry.getDouble(kRollerPIDs[0]), rollerIEntry.getDouble(kRollerPIDs[1]),
                    rollerDEntry.getDouble(kRollerPIDs[2]));
            rollerModuleIO.setFF(rollerSEntry.getDouble(kRollerSVAs[0]), rollerVEntry.getDouble(kRollerSVAs[1]),
                    rollerAEntry.getDouble(kRollerSVAs[2]));

            if (armExtendEntry.getBoolean(false)) {
                armModuleIO.extend();
            }

            if (armRetractEntry.getBoolean(false)) {
                armModuleIO.retract();
            }

            if (rollerIntakeEntry.getBoolean(false)) {
                rollerModuleIO.intake();
            }

            if (rollerOuttakeEntry.getBoolean(false)) {
                rollerModuleIO.outtake();
            }

            if (rollerStopEntry.getBoolean(false)) {
                rollerModuleIO.stop();
            }
        }

        Logger.recordOutput("Intake/Arm/IsExetended", armModuleIO.isExtended());
        Logger.recordOutput("Intake/Arm/IsRetracted", armModuleIO.isRetracted());
        Logger.recordOutput("Intake/Arm/CurrentAngle", armModuleIO.getPosition());
        Logger.recordOutput("Intake/Arm/SetPointAngle", armModuleIO.getGoalPosition());
        Logger.recordOutput("Intake/Roller/CurrentRPM", rollerModuleIO.getVelocity());
        Logger.recordOutput("Intake/Roller/SetPointRPM", rollerModuleIO.getGoalVelocity());
    }

    public void extendArm() {
        armModuleIO.extend();
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
        rollerModuleIO.intake();
    }

    public void outtakeRoller() {
        rollerModuleIO.outtake();
    }

    public void retractArm() {
        armModuleIO.retract();
    }

    public void setArmPID(boolean armPIDEnabled) {
        this.armPIDEnabled = armPIDEnabled;
    }

    public void setRollerPID(boolean rollerPIDEnabled) {
        this.rollerPIDEnabled = rollerPIDEnabled;
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

    /**
     * 
     */
    private void setShuffleboard() {
        armTab = Shuffleboard.getTab("Intake/Arm");
        armPEntry = armTab.add("Set kP", kArmPIDs[0]).getEntry();
        armIEntry = armTab.add("Set kI", kArmPIDs[1]).getEntry();
        armDEntry = armTab.add("Set kD", kArmPIDs[2]).getEntry();

        armPEntry.setDouble(kArmPIDs[0]);
        armIEntry.setDouble(kArmPIDs[1]);
        armDEntry.setDouble(kArmPIDs[2]);

        armSEntry = armTab.add("Set kS", kArmSVAs[0]).getEntry();
        armVEntry = armTab.add("Set kV", kArmSVAs[1]).getEntry();
        armAEntry = armTab.add("Set kA", kArmSVAs[2]).getEntry();

        armSEntry.setDouble(kArmSVAs[0]);
        armVEntry.setDouble(kArmSVAs[1]);
        armAEntry.setDouble(kArmSVAs[2]);

        armRetractEntry = armTab.add("Force Retract", false)
                .withWidget("Toggle Button")
                .getEntry();
        armExtendEntry = armTab.add("Force Extend", false)
                .withWidget("Toggle Button")
                .getEntry();

        rollerTab = Shuffleboard.getTab("Intake/Roller");
        rollerPEntry = rollerTab.add("Set kP", kRollerPIDs[0]).getEntry();
        rollerIEntry = rollerTab.add("Set kI", kRollerPIDs[1]).getEntry();
        rollerDEntry = rollerTab.add("Set kD", kRollerPIDs[2]).getEntry();

        rollerPEntry.setDouble(kRollerPIDs[0]);
        rollerIEntry.setDouble(kRollerPIDs[1]);
        rollerDEntry.setDouble(kRollerPIDs[2]);

        rollerSEntry = rollerTab.add("Set kS", kRollerSVAs[0]).getEntry();
        rollerVEntry = rollerTab.add("Set kV", kRollerSVAs[1]).getEntry();
        rollerAEntry = rollerTab.add("Set kA", kRollerSVAs[2]).getEntry();

        rollerSEntry.setDouble(kRollerSVAs[0]);
        rollerVEntry.setDouble(kRollerSVAs[1]);
        rollerAEntry.setDouble(kRollerSVAs[2]);

        rollerIntakeEntry = rollerTab.add("Force Intake", false)
                .withWidget("Toggle Switch")
                .getEntry();
        rollerOuttakeEntry = rollerTab.add("Force Outtake", false)
                .withWidget("Toggle Switch")
                .getEntry();
        rollerStopEntry = rollerTab.add("Stop Motor", false)
                .withWidget("Toggle Switch")
                .getEntry();
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
