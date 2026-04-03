package frc.robot.commands;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.AutoBuilderConstants;
import frc.robot.Constants.ClimbConstants;
import frc.robot.subsystems.climb.ClimbSubsystem;
import frc.robot.util.GameUtil;

/**
 * 
 */
public class ClimbToBarCommand extends Command {
    // Subsystems
    private final ClimbSubsystem climbSubsystem;
    private final Supplier<Pose2d> robotPoseSupplier;

    // Event triggers
    private final EventTrigger climbExtend;
    private final EventTrigger climbRetract;

    // Variables
    private Command followPathCommand;

    /**
     * 
     */
    public ClimbToBarCommand(ClimbSubsystem climbSubsystem, Supplier<Pose2d> robotPoseSupplier) {
        this.climbSubsystem = climbSubsystem;
        this.robotPoseSupplier = robotPoseSupplier;

        this.climbExtend = new EventTrigger("ClimbExtend");
        this.climbRetract = new EventTrigger("ClimbRetract");

        addRequirements(climbSubsystem);
    }

    @Override
    public void initialize() {
        followPathCommand = getFollowPathCommand();

        if (followPathCommand != null) {
            followPathCommand.initialize();

            // define the event triggers
            climbExtend.onTrue(
                    Commands.runOnce(() -> climbSubsystem.setPosition(ClimbConstants.EXTEND_LENGTH_INCHES, false)));
            climbRetract.onTrue(
                    Commands.runOnce(() -> climbSubsystem.setPosition(ClimbConstants.RETRACT_LENGTH_INCHES, true)));

            Logger.recordOutput("Commands/Active Command", this.getName());
        }
    }

    @Override
    public void execute() {
        if (followPathCommand != null) {
            followPathCommand.execute();

            Logger.recordOutput("PathPlanner/Events/ExtendMarkder", climbExtend.getAsBoolean());
            Logger.recordOutput("PathPlanner/Events/RetractMarker", climbRetract.getAsBoolean());
        }
    }

    @Override
    public void end(boolean interrupted) {
        if (followPathCommand != null) {
            followPathCommand.end(interrupted);

            if (!interrupted)
                climbSubsystem.holdPosition();

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
            PathPlannerPath pathPlannerPath = PathPlannerPath.fromPathFile(getNearestTowerBarPath());

            return AutoBuilder.pathfindThenFollowPath(pathPlannerPath, AutoBuilderConstants.CONSTRAINTS);
        } catch (Exception e) {
            return null;
        }
    }

    /**
     * 
     */
    private String getNearestTowerBarPath() {
        Pose2d robotPose = robotPoseSupplier.get();
        boolean isAboveMidline = GameUtil.isAboveMidLine(robotPose);

        return isAboveMidline ? "Climb Right Bar Path" : "Climb Left Bar Path";
    }
}
