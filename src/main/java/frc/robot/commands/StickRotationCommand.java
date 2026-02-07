package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.containers.RobotContainer;
import frc.robot.subsystems.Turret;

public class StickRotationCommand extends Command {

    private final Turret turretSubsystem;
    private final CommandXboxController joystick ;

    
    public StickRotationCommand(Turret turretSubsystem, CommandXboxController joystick) {
        this.turretSubsystem = turretSubsystem;
        this.joystick = joystick;

        addRequirements(turretSubsystem);
    }

    public void initialize() {
        turretSubsystem.setIdle();
    }
    
    public void execute() {
        double rightX = joystick.getRawAxis(turretSubsystem.rotationAxis);
        turretSubsystem.runTurret(rightX * 0.1);
    }

}
