package frc.robot.commands.Intake;

import java.util.function.Supplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeSubsystem;

/**
 * Varies intake speed based on robot velocity
 * - Slower robot = slower intake (min 1500 RPM)
 * - Faster robot = faster intake (max 3000 RPM)
 * - Prevents balls from flying out at slow speeds
 */
public class AdaptiveIntakeCommand extends Command {
    private final IntakeSubsystem m_intake;
    private final Supplier<ChassisSpeeds> m_chassisSpeeds;

    private static final double MIN_INTAKE_RPM = 700;
    private static final double MAX_INTAKE_RPM = 3000;
    private static final double MAX_ROBOT_SPEED = 4.5; // m/s

    public AdaptiveIntakeCommand(IntakeSubsystem intake, Supplier<ChassisSpeeds> chassisSpeeds) {
        m_intake = intake;
        m_chassisSpeeds = chassisSpeeds;
        addRequirements(intake);
    }

    @Override
    public void execute() {
        // Get current robot speed (magnitude of velocity)
        ChassisSpeeds speeds = m_chassisSpeeds.get();
        double robotSpeed = Math.sqrt(speeds.vxMetersPerSecond * speeds.vxMetersPerSecond +
                speeds.vyMetersPerSecond * speeds.vyMetersPerSecond);

        // Map robot speed to intake RPM
        // At 0 m/s -> 1500 RPM
        // At 4.5 m/s -> 3000 RPM
        double intakeRPM = MIN_INTAKE_RPM + (robotSpeed / MAX_ROBOT_SPEED) * (MAX_INTAKE_RPM - MIN_INTAKE_RPM);

        // Clamp to min/max
        intakeRPM = Math.max(MIN_INTAKE_RPM, Math.min(MAX_INTAKE_RPM, intakeRPM));

        // Set the roller speed via PID
        m_intake.setRollerPID(true);
        m_intake.setRollerDesiredSpeed(intakeRPM);
    }

    @Override
    public void end(boolean interrupted) {
        m_intake.setRollerPID(false);
        m_intake.setRollerSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
