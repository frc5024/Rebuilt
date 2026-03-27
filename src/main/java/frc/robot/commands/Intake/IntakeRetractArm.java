package frc.robot.commands.Intake;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.intakeConstants;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class IntakeRetractArm extends Command {
    private IntakeSubsystem s_Intake;

    static ShuffleboardTab tab = Shuffleboard.getTab("intakeMotor");
    static GenericEntry pEntry = tab.add("SET RETRACTING SPEED", intakeConstants.RETRACTING_SPEED).getEntry();

    public IntakeRetractArm(IntakeSubsystem s_Intake) {
        this.s_Intake = s_Intake;
    }

    @Override
    public void initialize() {
        s_Intake.setArmSpeed(pEntry.getDouble(intakeConstants.RETRACTING_SPEED));
    }

    @Override
    public void execute() {
        s_Intake.setArmSpeed(pEntry.getDouble(intakeConstants.RETRACTING_SPEED));
    }

    @Override
    public void end(boolean interrupted) {
        s_Intake.setArmSpeed(0.0);
    }

    @Override
    public boolean isFinished() {
        return s_Intake.isIntakeRetracted();
    }
}