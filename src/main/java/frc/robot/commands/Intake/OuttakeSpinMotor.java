package frc.robot.commands.Intake;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.intakeConstants;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class OuttakeSpinMotor extends Command {
    private IntakeSubsystem s_Intake;
    double speed = 0.6;

    static ShuffleboardTab tab = Shuffleboard.getTab("intakeMotor");
    static GenericEntry pEntry = tab.add("SET OUTTAKESPEED", intakeConstants.OUTTAKE_SPEED).getEntry();

    public OuttakeSpinMotor(IntakeSubsystem s_Intake) {
        this.s_Intake = s_Intake;
        addRequirements(s_Intake);
    }

    @Override
    public void initialize() {
        s_Intake.setIntakeSpeed(pEntry.getDouble(0.1));
    }

    @Override
    public void end(boolean interrupted) {
        s_Intake.setIntakeSpeed(0.0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
