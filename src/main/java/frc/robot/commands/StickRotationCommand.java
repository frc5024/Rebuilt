package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.turret.TurretSubsystem;

/**
 * 
 */
public class StickRotationCommand extends Command {
    // Subsystems
    private final TurretSubsystem turretSubsystem;

    // Variables
    public double speed;

    /**
     * 
     */
    public StickRotationCommand(TurretSubsystem turretSubsystem, double speed) {
        this.turretSubsystem = turretSubsystem;
        this.speed = speed;

        addRequirements(turretSubsystem);
    }

    @Override
    public void initialize() {
        turretSubsystem.disablePID();
    }

    @Override
    public void execute() {
        turretSubsystem.runTurret(speed);

        Logger.recordOutput("Commands/Active Command", this.getName());
    }

    @Override
    public void end(boolean interrupted) {
        turretSubsystem.disablePID();

        Logger.recordOutput("Commands/Active Command", "");
    }
}
