package frc.robot.commands;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.turret.TurretSubsystem;

public class turretSweepCommand extends Command {

    private final TurretSubsystem turretSubsystem;
    private double targetAngle;

    static ShuffleboardTab tab = Shuffleboard.getTab("turretSweep");

    // CommandXboxController operator = RobotContainer.operator;

    public turretSweepCommand(TurretSubsystem turretSubsystem) {
        this.turretSubsystem = turretSubsystem;
        addRequirements(turretSubsystem);
    }

    public void initialize() {

        turretSubsystem.setTargetAngle(-150);

        turretSubsystem.enablePID();

        // if (turretSubsystem.getHallEffect() == false) {
        // turretSubsystem.setTargetAngle(0);
        // System.out.println("set to 0");
        // turretSubsystem.zeroEncoder();
        // } else {
        // turretSubsystem.setTargetAngle(150);
        // System.out.println("set to 150");
        // }
    }

    public void execute() {
        if (turretSubsystem.isAtTargetAngle() == true) {
            turretSubsystem.disablePID();
            turretSubsystem.setTargetAngle(150);
            turretSubsystem.enablePID();

            if (turretSubsystem.isAtTargetAngle() == true && turretSubsystem.getHallEffect() == false) {
                turretSubsystem.disablePID();
                turretSubsystem.setTargetAngle(0);
                turretSubsystem.enablePID();
            }
        }

    }

    public boolean isFinished() {
        // if (turretSubsystem.isAtTargetAngle() == true) {
        // System.out.println("AT SETPOINT");
        // }
        // return turretSubsystem.isAtTargetAngle();
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        turretSubsystem.disablePID();
        // Optionally stop motor explicitly
        turretSubsystem.setIdle();
    }

}
