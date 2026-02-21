package frc.robot.commands.Turret;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.containers.RobotContainer;
import frc.robot.subsystems.turret.TurretSubsystem;

public class StickRotationCommand extends Command {

    private final TurretSubsystem turretSubsystem;
    public double speed;

    
    public StickRotationCommand(TurretSubsystem turretSubsystem, double speed) {
        this.turretSubsystem = turretSubsystem;
        this.speed = speed;

        addRequirements(turretSubsystem);
    }

    public void initialize() {
        turretSubsystem.setIdle();
    }
    
    public void execute() {
        turretSubsystem.runTurret(speed);
    }

  @Override
  public void end(boolean interrupted) {
    turretSubsystem.setIdle();
  }

}
