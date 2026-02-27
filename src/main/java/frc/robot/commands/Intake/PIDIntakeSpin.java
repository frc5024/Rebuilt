package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class PIDIntakeSpin extends Command {
    public final IntakeSubsystem m_IntakeSubsystem;

    public PIDIntakeSpin(IntakeSubsystem m_IntakeSubsystem) {
        this.m_IntakeSubsystem = m_IntakeSubsystem;
    }
}
