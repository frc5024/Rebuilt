package frc.robot.commands;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.intakeConstants;
import frc.robot.subsystems.Intake;

public class IntakeStopArm extends Command {
    private Intake s_Intake;
    //double speed = 0.6;

    ShuffleboardTab tab = Shuffleboard.getTab("intakeMotor");
    GenericEntry pEntry = tab.add("STOP", intakeConstants.stopArm).getEntry();
    
    public IntakeStopArm(Intake s_Intake) {
        this.s_Intake = s_Intake;
        addRequirements (s_Intake);
    }

    @Override
    public void initialize() {
        s_Intake.setIntakeSpeed(pEntry.getDouble(0.0));
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