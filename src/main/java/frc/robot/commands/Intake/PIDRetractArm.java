package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class PIDRetractArm extends Command {
    public final IntakeSubsystem m_IntakeSubsystem;

    public PIDRetractArm(IntakeSubsystem m_IntakeSubsystem) {
        this.m_IntakeSubsystem = m_IntakeSubsystem;
    }

    @Override
    public void initialize() {
        m_IntakeSubsystem.setArmSetVelocity(-100);
        m_IntakeSubsystem.setArmPID(true);
    }

    @Override
    public void end(boolean interrupted) {
        m_IntakeSubsystem.setRollerPID(false);
    }

    public boolean isFinished() {
        return m_IntakeSubsystem.isIntakeRetracted();
    }
}