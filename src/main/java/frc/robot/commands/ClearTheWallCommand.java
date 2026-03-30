package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.SwerveDriveSubsystem;

/**
 * 
 */
public class ClearTheWallCommand extends Command {
    // Subsystems
    private final SwerveDriveSubsystem swerveDriveSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private final ShooterSubsystem shooterSubsystem;
    private final HopperSubsystem hopperSubsystem;
    private final FeederSubsystem feederSubsystem;

    // Event triggers
    private final EventTrigger extendAndIntakeTrigger;
    private final EventTrigger shootTrigger;

    // Variables
    private final PathConstraints CONSTRAINTS = new PathConstraints(2.0, 4.0, Units.degreesToRadians(540),
            Units.degreesToRadians(720));
    private Command followPathCommand;

    /**
     * 
     */
    public ClearTheWallCommand(SwerveDriveSubsystem swerveDriveSubsystem, IntakeSubsystem intakeSubsystem,
            ShooterSubsystem shooterSubsystem, HopperSubsystem hopperSubsystem, FeederSubsystem feederSubsystem) {
        this.swerveDriveSubsystem = swerveDriveSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.hopperSubsystem = hopperSubsystem;
        this.feederSubsystem = feederSubsystem;

        this.extendAndIntakeTrigger = new EventTrigger("CTW-ExtendAndIntake");
        this.shootTrigger = new EventTrigger("CTW-Shoot");

        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        followPathCommand = getFollowPathCommand();

        if (this.followPathCommand != null) {
            this.followPathCommand.initialize();

            // define the event triggers
            extendAndIntakeTrigger.onTrue(
                    Commands.sequence(
                            Commands.runOnce(() -> intakeSubsystem.intakeRoller()),
                            Commands.runOnce(() -> intakeSubsystem.extendArm())));

            shootTrigger.whileTrue(new ShootCommand(swerveDriveSubsystem, shooterSubsystem,
                    hopperSubsystem, feederSubsystem, () -> swerveDriveSubsystem.getPose()));

            Logger.recordOutput("Commands/Active Command", this.getName());
        }
    }

    @Override
    public void execute() {
        if (followPathCommand != null) {
            followPathCommand.execute();

            Logger.recordOutput("PathPlanner/Events/ExtendAndIntakeMarker", extendAndIntakeTrigger.getAsBoolean());
            Logger.recordOutput("PathPlanner/Events/ShootMarker", shootTrigger.getAsBoolean());
        }
    }

    @Override
    public void end(boolean interrupted) {
        if (followPathCommand != null) {
            followPathCommand.end(interrupted);

            Logger.recordOutput("Commands/Active Command", "");
        }
    }

    @Override
    public boolean isFinished() {
        return followPathCommand != null ? followPathCommand.isFinished() : true;
    }

    /**
     * 
     */
    private Command getFollowPathCommand() {
        try {
            PathPlannerPath pathPlannerPath = PathPlannerPath.fromPathFile("Clear The Wall Path");

            return AutoBuilder.pathfindThenFollowPath(pathPlannerPath, CONSTRAINTS);
        } catch (Exception e) {
            return null;
        }
    }
}
