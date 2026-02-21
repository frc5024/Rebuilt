package frc.robot.commands.Turret;

import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.containers.RobotContainer;
<<<<<<< HEAD
import frc.robot.subsystems.Turret;
=======
import frc.robot.subsystems.turret.TurretSubsystem;
>>>>>>> 4035f3e568ecd0b77ffcdba7f64a940fd2a9cc00
import frc.robot.Constants;

public class spinCommand extends Command {

<<<<<<< HEAD
    private final Turret turretSubsystem;

    // CommandXboxController operator = RobotContainer.operator;
    
    public spinCommand(Turret turretSubsystem) {
=======
    private final TurretSubsystem turretSubsystem;

    // CommandXboxController operator = RobotContainer.operator;
    
    public spinCommand(TurretSubsystem turretSubsystem) {
>>>>>>> 4035f3e568ecd0b77ffcdba7f64a940fd2a9cc00
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
