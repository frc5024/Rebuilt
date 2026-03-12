package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.turret.TurretSubsystem;

/**
 * 
 */
public class turretSweepCommand extends Command {
    // Subsystems
    private final TurretSubsystem turretSubsystem;

    /**
     * 
     */
    public turretSweepCommand(TurretSubsystem turretSubsystem) {
        this.turretSubsystem = turretSubsystem;

        addRequirements(turretSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if (!turretSubsystem.getHallEffect()) {
            turretSubsystem.setPosition(135);
            turretSubsystem.setAngle(0);
            turretSubsystem.enablePID();
        } else if (turretSubsystem.getHallEffect() && !turretSubsystem.isPIDEnabled()) {
            turretSubsystem.runTurret(0.05);
        }

        Logger.recordOutput("Commands/Active Command", this.getName());
    }

    @Override
    public boolean isFinished() {
        return turretSubsystem.atGoal();
    }

    @Override
    public void end(boolean interrupted) {
        turretSubsystem.disablePID();

        Logger.recordOutput("Commands/Active Command", "");
    }
}
