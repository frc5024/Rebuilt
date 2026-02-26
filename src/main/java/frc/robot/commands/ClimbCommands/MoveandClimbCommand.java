// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ClimbCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.climbConstants;
import frc.robot.commands.PathFinderAndFollowCommand;
import frc.robot.subsystems.climb.ClimbSubsystem;
import frc.robot.subsystems.swervedrive.SwerveDriveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveandClimbCommand extends Command {
    private final ClimbSubsystem m_Climb;
    private final SwerveDriveSubsystem m_SwerveDriveSubsystem;
    public final Timer m_timer = new Timer();
    private PathFinderAndFollowCommand m_pathfinder;
    private final String toClimb = "paths/MoveandClimb.wpilib.json";

    /** Creates a new MoveandClimbCommand. */
    public MoveandClimbCommand(ClimbSubsystem climb, SwerveDriveSubsystem swerveDriveSubsystem) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.m_Climb = climb;
        this.m_SwerveDriveSubsystem = swerveDriveSubsystem;
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

        m_Climb.setSpeed(climbConstants.contractSpeed);
        new SequentialCommandGroup(null)

                // .andThen(new ParallelCommandGroup(
                // new ContractCommand(m_Climb),
                // new PathFinderAndFollowCommand(m_SwerveDriveSubsystem, toClimb)))
                .addCommands(new PathFinderAndFollowCommand(m_SwerveDriveSubsystem, toClimb));
        // .addCommands(new ClimbCommand(this.m_Climb));
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_Climb.setSpeed(0.0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
