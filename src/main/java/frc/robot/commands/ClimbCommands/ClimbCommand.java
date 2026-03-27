// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ClimbCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.climbConstants;
import frc.robot.subsystems.climb.ClimbSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ClimbCommand extends Command {
    private final ClimbSubsystem m_Climb;
    public final Timer m_timer = new Timer();

    /** Creates a new ClimCommand. */
    public ClimbCommand(ClimbSubsystem climb) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.m_Climb = climb;
        addRequirements(m_Climb);

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_Climb.setSpeed(0.0);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_Climb.setSpeed(climbConstants.climbspeed);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_Climb.setSpeed(0.0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // Ends the command when the climb is fully contracted
        return m_Climb.value() < climbConstants.minPos;
    }
}
