package frc.robot.commands;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.intakeConstants;
import frc.robot.subsystems.Intake;

public class IntakeExtendArm extends Command {
    private Intake s_Intake;
    double speed = 0.6;

    ShuffleboardTab tab = Shuffleboard.getTab("intakeMotor");
    GenericEntry pEntry = tab.add("SET EXTENDING SPEED", intakeConstants.extendingSpeed).getEntry();

    public IntakeExtendArm(Intake s_Intake) {
        this.s_Intake = s_Intake;
        addRequirements (s_Intake);
    }

    @Override
    public void initialize() {
        s_Intake.setArmSpeed(pEntry.getDouble(-0.1));
    }

    @Override
    public void end(boolean interrupted) {
        s_Intake.setArmSpeed(0.0);
    }

    @Override
    public boolean isFinished() {
    return s_Intake.isIntakeExtended();
  }


}