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
    public double speed;

    
    public StickRotationCommand(Turret turretSubsystem, double speed) {
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
