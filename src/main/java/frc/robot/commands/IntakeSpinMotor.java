package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class IntakeSpinMotor extends Command {
    private Intake s_Intake;


    public IntakeSpinMotor(Intake s_Intake) {
        this.s_Intake = s_Intake;
        addRequirements (s_Intake);
    }

    @Override
    public void initialize() {
        s_Intake.setSpeed(4);
    }
}
