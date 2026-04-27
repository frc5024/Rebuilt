package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.containers.RebuiltRobotContainer;
import frc.robot.subsystems.turret.TurretSubsystem;

public class StickRotationCommand extends Command {

    private final TurretSubsystem turretSubsystem;

    CommandXboxController operator = RebuiltRobotContainer.operatorController;

    public StickRotationCommand(TurretSubsystem turretSubsystem) {
        this.turretSubsystem = turretSubsystem;

        addRequirements(turretSubsystem);
    }

    public void initialize() {
        turretSubsystem.runTurret(0);
    }

    public void execute() {
        double rightX = operator.getRawAxis(turretSubsystem.rotationAxis);
        turretSubsystem.runTurret(rightX * 0.1);
    }

}