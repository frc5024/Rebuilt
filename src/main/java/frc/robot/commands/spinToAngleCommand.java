package frc.robot.commands;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.turret.TurretSubsystem;

public class spinToAngleCommand extends Command {

    private final TurretSubsystem turretSubsystem;
    private double targetAngle;

    static ShuffleboardTab tab = Shuffleboard.getTab("spintoangle");
    static GenericEntry angleEntry = tab.add("set target angle", 0).getEntry();

    // CommandXboxController operator = RobotContainer.operator;

    public spinToAngleCommand(TurretSubsystem turretSubsystem, double angle) {
        this.turretSubsystem = turretSubsystem;
        this.targetAngle = angle;
        addRequirements(turretSubsystem);
    }

    public void initialize() {
        System.out.println("setting turret angle");

        targetAngle = angleEntry.getDouble(0);
        // turretSubsystem.zeroEncoder();
        turretSubsystem.setTargetAngle(targetAngle);
        turretSubsystem.enablePID();
    }

    public void execute() {
        // System.out.println(turretSubsystem.getTurretAngle());
        // turretSubsystem.updateTurretAngle();

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
