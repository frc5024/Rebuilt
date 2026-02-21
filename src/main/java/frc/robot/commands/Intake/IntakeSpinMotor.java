package frc.robot.commands.Intake;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.intakeConstants;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class IntakeSpinMotor extends Command {
    private final IntakeSubsystem s_Intake;

   static ShuffleboardTab tab = Shuffleboard.getTab("intakeMotor");
    static GenericEntry intakeEntry = tab.add("SET INTAKESPEED", intakeConstants.INTAKE_SPEED).getEntry();

    public IntakeSpinMotor(IntakeSubsystem s_Intake) {
        this.s_Intake = s_Intake;
    }

    @Override
    public void initialize() {
        s_Intake.setIntakeSpeed(intakeEntry.getDouble(-0.1));
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
