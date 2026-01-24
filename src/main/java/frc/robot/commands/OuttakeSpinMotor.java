package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;


public class OuttakeSpinMotor extends Command {
    private Intake s_Intake;
    double speed = 0.6;

    public OuttakeSpinMotor(Intake s_Intake) {
        this.s_Intake = s_Intake;
        addRequirements (s_Intake);
    }

    @Override
    public void initialize() {
        s_Intake.setSpeed(speed);
    }

    @Override
    public void end(boolean interrupted) {
        s_Intake.setSpeed(0.0);
    }
}

