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

    // Variables
    private boolean hitHallEffect;

    /**
     * 
     */
    public turretSweepCommand(TurretSubsystem turretSubsystem) {
        this.turretSubsystem = turretSubsystem;

        addRequirements(turretSubsystem);
    }

    @Override
    public void initialize() {
        hitHallEffect = false;
    }

    @Override
    public void execute() {
        // manually run the turret until it hits the hall effect sensor
        if (turretSubsystem.getHallEffectValue() && !turretSubsystem.isPIDEnabled()) {
            turretSubsystem.runTurret(0.5);
        } else if (!turretSubsystem.getHallEffectValue()) {
            // now use pid to center the turret
            turretSubsystem.setAngle(0);
            turretSubsystem.enablePID();
            hitHallEffect = true;
        }

        Logger.recordOutput("Turret/Active Command", this.getName());
    }

    @Override
    public boolean isFinished() {
        return hitHallEffect && turretSubsystem.atGoal();
    }

    @Override
    public void end(boolean interrupted) {
        turretSubsystem.disablePID();

        Logger.recordOutput("Turret/Active Command", "");
    }
}
