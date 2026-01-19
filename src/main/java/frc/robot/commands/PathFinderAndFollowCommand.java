package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.swervedrive.SwerveDriveSubsystem;

import org.littletonrobotics.junction.Logger;

/** A command that runs pathfindThenFollowPath based on the current drive mode. */
public class PathFinderAndFollowCommand extends Command {
  private final SwerveDriveSubsystem swerveDrive;
  private final String pathName;

  private Command commandGroup;
  private Command followPathCommand;

  /**
   * Creates a new PathFinderAndFollow command.
   *
   * @param stationModeSupplier a supplier for the drive mode type
   */
  public PathFinderAndFollowCommand(SwerveDriveSubsystem swerveDrive, String pathName) {
    this.swerveDrive = swerveDrive;
    this.pathName = pathName;
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    if (this.commandGroup != null) this.commandGroup.cancel();

    Logger.recordOutput("Commands/Active Command", "");
  }

  @Override
  public void execute() {
    if (this.followPathCommand != null) {
      this.followPathCommand.execute();
    }
  }

  @Override
  public void initialize() {
    // zero drive pid since we are driving closed loop

    this.followPathCommand = getfollowPathCommand();
    if (this.followPathCommand != null) {

      this.followPathCommand.initialize();
      Logger.recordOutput("Commands/Active Command", this.getName());
    }
  }

  @Override
  public boolean isFinished() {
    return this.followPathCommand != null ? this.followPathCommand.isFinished() : true;
  }

  /** Runs a new autonomous path based on the current drive mode. */
  private Command getfollowPathCommand() {
    try {

      PathPlannerPath pathPlannerPath = PathPlannerPath.fromPathFile("Example Path");

      return AutoBuilder.pathfindThenFollowPath(pathPlannerPath, Constants.CONSTRAINTS);

    } catch (Exception e) {
      return null;
    }
  }
}
