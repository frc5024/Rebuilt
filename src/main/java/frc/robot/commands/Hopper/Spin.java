package frc.robot.commands.Hopper;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hopper;

public class Spin extends Command{
    public Hopper hopperSubsystem;
    public double speed;

    public Spin(Hopper hopperSubsytem, double speed) {
        this.hopperSubsystem = hopperSubsytem;
        this.speed = speed;
    }

    @Override
    public void initialize() {
        hopperSubsystem.setIdle();
    }

    @Override
    public void execute() {
        hopperSubsystem.setSpeed(speed);
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

