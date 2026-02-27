package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.intakeConstants;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class PIDExtendArm extends Command {
    public final IntakeSubsystem m_IntakeSubsystem;

    public PIDExtendArm(IntakeSubsystem m_IntakeSubsystem) {
        this.m_IntakeSubsystem = m_IntakeSubsystem;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        System.out.println("Extending");
        m_IntakeSubsystem.setArmSetVelocity(intakeConstants.EXTENDING_SPEED);
        m_IntakeSubsystem.setArmPID(true);
    }

    @Override
    public void end(boolean interrupted) {
        m_IntakeSubsystem.setArmPID(false);
    }

    @Override
    public boolean isFinished() {
        return m_IntakeSubsystem.isIntakeExtended();
    }
}
