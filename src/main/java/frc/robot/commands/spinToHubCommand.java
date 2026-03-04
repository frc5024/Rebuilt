package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.turret.TurretSubsystem;

public class spinToHubCommand extends Command {

    private final TurretSubsystem turretSubsystem;
    private final double targetAngle;

    // CommandXboxController operator = RobotContainer.operator;

    public spinToHubCommand(TurretSubsystem turretSubsystem, double angle) {
        this.turretSubsystem = turretSubsystem;
        this.targetAngle = angle;
        addRequirements(turretSubsystem);
    }

    public void initialize() {
        System.out.println("setting turret angle");
        // turretSubsystem.resetPID(); // turretSubsystem.zeroEncoder();
        turretSubsystem.setTargetAngle(targetAngle);
        turretSubsystem.enablePID();
    }

    public void execute() {

        // System.out.println(turretSubsystem.getTurretAngle());
        // turretSubsystem.updateTurretAngle();

    }

    public boolean isFinished() {
        if (turretSubsystem.isAtTargetAngle() == true) {
            System.out.println("AT SETPOINT");
        }
        return turretSubsystem.isAtTargetAngle();
    }

    @Override
    public void end(boolean interrupted) {
        turretSubsystem.disablePID();
        // Optionally stop motor explicitly
        turretSubsystem.setIdle();
    }

}
