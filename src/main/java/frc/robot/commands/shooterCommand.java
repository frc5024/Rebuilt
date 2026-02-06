package frc.robot.commands;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.shooterConstants;
import frc.robot.subsystems.Shooter;

public class shooterCommand extends Command {
    private final Shooter shooterSubsystem;

    static ShuffleboardTab tab = Shuffleboard.getTab("shooterMotor");
    static GenericEntry pEntry = tab.add("SET SHOOTING SPEED", shooterConstants.speed).getEntry();

    public shooterCommand(Shooter shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {
        shooterSubsystem.setIdle();
    }

    @Override
    public void execute() {
        shooterSubsystem.setMotorSpeed(pEntry.getDouble(0.1));
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.setIdle();
    }
}
