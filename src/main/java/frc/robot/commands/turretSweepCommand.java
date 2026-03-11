package frc.robot.commands;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.turret.TurretSubsystem;

public class turretSweepCommand extends Command {

    private final TurretSubsystem turretSubsystem;
    private boolean sweepDone;

    ShuffleboardTab tab = Shuffleboard.getTab("turretSweep");

    // CommandXboxController operator = RobotContainer.operator;

    public turretSweepCommand(TurretSubsystem turretSubsystem) {
        this.turretSubsystem = turretSubsystem;
        addRequirements(turretSubsystem);

        tab.addBoolean("Sweep Done", () -> sweepDone);
        tab.addBoolean("isattargetangle", () -> turretSubsystem.isAtTargetAngle());
    }

    public void initialize() {

    }

    public void execute() {
        if (!turretSubsystem.getHallEffect()) {
            turretSubsystem.setPosition(135);
            turretSubsystem.setTargetAngle(0);
            turretSubsystem.enablePID();
        } else if (turretSubsystem.getHallEffect() && !turretSubsystem.pidEnabled) {
            turretSubsystem.runTurret(0.05);
        }

    }

    public boolean isFinished() {
        if (turretSubsystem.isAtTargetAngle()) {
            // turretSubsystem.setPosition(0);
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        turretSubsystem.disablePID();
        sweepDone = true;
        turretSubsystem.setIdle();
    }

}
