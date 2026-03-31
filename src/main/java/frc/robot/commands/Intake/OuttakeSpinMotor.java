package frc.robot.commands.Intake;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.intakeConstants;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class OuttakeSpinMotor extends Command {
    private final IntakeSubsystem s_Intake;

    static ShuffleboardTab tab = Shuffleboard.getTab("intakeMotor");
    static GenericEntry intakeEntry = tab.add("SET Outtake SPeed", intakeConstants.OUTTAKE_SPEED).getEntry();

    public OuttakeSpinMotor(IntakeSubsystem s_Intake) {
        this.s_Intake = s_Intake;
    }

    @Override
    public void initialize() {
        s_Intake.setRollerSpeed(intakeEntry.getDouble(intakeConstants.OUTTAKE_SPEED));
    }

    @Override
    public void execute() {
        s_Intake.setRollerSpeed(intakeEntry.getDouble(intakeConstants.OUTTAKE_SPEED));
    }

    @Override
    public void end(boolean interrupted) {
        s_Intake.setRollerSpeed(0.0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
