package frc.robot.commands.turret;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.containers.RobotContainer;
import frc.robot.subsystems.turret.TurretSubsystem;

public class StickRotationCommand extends Command {

    private final TurretSubsystem turretSubsystem;

    CommandXboxController driver = RobotContainer.driverController;
    
    public  StickRotationCommand(TurretSubsystem turretSubsystem) {
        this.turretSubsystem = turretSubsystem;

        addRequirements(turretSubsystem);
    }

    public void initialize() {
        turretSubsystem.setIdle();
    }
    
    public void execute() {
        double rightX = driver.getRawAxis(turretSubsystem.rotationAxis);
        turretSubsystem.runTurret(rightX * 0.1);
    }

}
