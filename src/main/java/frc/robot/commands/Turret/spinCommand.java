package frc.robot.commands.Turret;

import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.containers.RobotContainer;
import frc.robot.subsystems.Turret;
import frc.robot.Constants;

public class spinCommand extends Command {

    private final Turret turretSubsystem;

    // CommandXboxController operator = RobotContainer.operator;
    
    public spinCommand(Turret turretSubsystem) {
        this.turretSubsystem = turretSubsystem;

        addRequirements(turretSubsystem);
    }

    public void initialize() {
        turretSubsystem.setIdle();
    }
    
    public void execute() {
        System.out.println("HIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIII");
        turretSubsystem.runTurret(Constants.turretConstants.speed);

    }

    public void end(boolean interrupted) {
        turretSubsystem.runTurret(0);
    }

}
