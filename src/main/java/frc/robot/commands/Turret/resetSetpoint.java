package frc.robot.commands.Turret;

import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.containers.RobotContainer;
import frc.robot.subsystems.Turret;
import frc.robot.Constants;

public class resetSetpoint extends Command {

    private final Turret turretSubsystem;
    //private final double targetAngle;

    // CommandXboxController operator = RobotContainer.operator;
    
    public resetSetpoint(Turret turretSubsystem) {
        this.turretSubsystem = turretSubsystem;
        //this.targetAngle = angle;
        addRequirements(turretSubsystem);
    }

    public void initialize() {
        //turretSubsystem.zeroEncoder();
        //turretSubsystem.setTargetAngle(targetAngle);
        //turretSubsystem.enablePID();
    }
    
    public void execute() {
        turretSubsystem.zeroEncoder();
        System.out.println("ENCODER HAS BEEN ZEROED");
        //System.out.println(turretSubsystem.getTurretAngle());
        // turretSubsystem.updateTurretAngle();

    }

    // public boolean isFinished() {
    //     System.out.println("AT SETPOINT");
    //     return turretSubsystem.isAtTargetAngle();
    // }

    // @Override
    // public void end(boolean interrupted) {
    //     turretSubsystem.disablePID();
    //     // Optionally stop motor explicitly
    //     turretSubsystem.setIdle();
    // }

}
