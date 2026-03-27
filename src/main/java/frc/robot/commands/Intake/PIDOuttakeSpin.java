package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.intakeConstants;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class PIDOuttakeSpin extends Command {
    public final IntakeSubsystem m_IntakeSubsystem;

    public PIDOuttakeSpin(IntakeSubsystem m_IntakeSubsystem) {
        this.m_IntakeSubsystem = m_IntakeSubsystem;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        m_IntakeSubsystem.setRollerSetVelocity(intakeConstants.OUTTAKE_SPEED);
        m_IntakeSubsystem.setRollerPID(true);
    }

    @Override
    public void end(boolean interrupted) {
        m_IntakeSubsystem.setRollerPID(false);
    }
}
