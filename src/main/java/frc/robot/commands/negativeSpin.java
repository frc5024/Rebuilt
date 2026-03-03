package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.turret.TurretSubsystem;

public class negativeSpin extends Command {

    private final TurretSubsystem turretSubsystem;
    public double speed;

    public negativeSpin(TurretSubsystem turretSubsystem, double speed) {
        this.turretSubsystem = turretSubsystem;
        this.speed = speed;

        addRequirements(turretSubsystem);
    }

    public void initialize() {
        turretSubsystem.setIdle();
    }

    public void execute() {
        turretSubsystem.runTurret(-0.3);
    }

    @Override
    public void end(boolean interrupted) {
        turretSubsystem.setIdle();
    }

}
