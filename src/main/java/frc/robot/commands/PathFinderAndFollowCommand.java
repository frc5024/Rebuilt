package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoBuilderConstants;

/**
 * A command that runs pathfindThenFollowPath based on the current drive mode.
 */
public class PathFinderAndFollowCommand extends Command {
    private final String pathName;

    private Command commandGroup;
    private Command followPathCommand;

    /**
     * Creates a new PathFinderAndFollow command.
     *
     * @param stationModeSupplier a supplier for the drive mode type
     */
    public PathFinderAndFollowCommand(String pathName) {
        this.pathName = pathName;
    }

    // End condition of the button being released
    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        if (this.commandGroup != null)
            this.commandGroup.cancel();

        Logger.recordOutput("Commands/Active Command", "");
    }

    @Override
    public void execute() {
        // Executes the command to follow the path if it is not null, otherwise does
        // nothing
        if (this.followPathCommand != null) {
            this.followPathCommand.execute();
        }
    }

    @Override
    public void initialize() {
        // zero drive pid since we are driving closed loop

        // Grabs the specified path and initializes the command to follow it
        this.followPathCommand = getfollowPathCommand();
        if (this.followPathCommand != null) {

            this.followPathCommand.initialize();
            Logger.recordOutput("Commands/Active Command", this.getName());
        }
    }

    // The command is finished when the path is finished or if there was an error
    // loading the path
    @Override
    public boolean isFinished() {
        return this.followPathCommand != null ? this.followPathCommand.isFinished() : true;
    }

    /** Runs a new autonomous path based on the current drive mode. */
    private Command getfollowPathCommand() {
        try {

            PathPlannerPath pathPlannerPath = PathPlannerPath.fromPathFile(pathName);

            return AutoBuilder.pathfindThenFollowPath(pathPlannerPath, AutoBuilderConstants.CONSTRAINTS);

        } catch (Exception e) {
            return null;
        }
    }
}
