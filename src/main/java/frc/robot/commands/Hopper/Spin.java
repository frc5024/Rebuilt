package frc.robot.commands.Hopper;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Hopper;

public class Spin extends Command{
    public Hopper hopperSubsystem;
    public double speed;

    static ShuffleboardTab tab = Shuffleboard.getTab("Hopper");
    static GenericEntry speedEntry = tab.add("SET HOPPERSPEED", Constants.HopperConstants.hopperSpeed).getEntry();

    public Spin(Hopper hopperSubsystem) {
        this.hopperSubsystem = hopperSubsystem;
    }

    @Override
    public void initialize() {
        hopperSubsystem.setIdle();
    }

    @Override
    public void execute() {
        hopperSubsystem.setSpeed(speedEntry.getDouble(0.1));
    }

    @Override
    public void end(boolean interrupted) {
        hopperSubsystem.setIdle();
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}

