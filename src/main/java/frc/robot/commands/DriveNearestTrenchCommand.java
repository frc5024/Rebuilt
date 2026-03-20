package frc.robot.commands;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.TeleopConstants;
import frc.robot.util.AllianceFlipUtil;

/**
 * 
 */
public class DriveNearestTrenchCommand extends Command {
    // Subsystems
    private final Supplier<Pose2d> robotPoseSupplier;

    // Variables
    private Command followPathCommand;

    /**
     * 
     */
    public DriveNearestTrenchCommand(Supplier<Pose2d> robotPoseSupplier) {
        this.robotPoseSupplier = robotPoseSupplier;
    }

    @Override
    public void initialize() {
        followPathCommand = getFollowPathCommand();

        if (this.followPathCommand != null) {
            this.followPathCommand.initialize();
            Logger.recordOutput("Commands/Active Command", this.getName());
        }
    }

    @Override
    public void execute() {
        if (followPathCommand != null) {
            followPathCommand.execute();
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
            PathPlannerPath pathPlannerPath = PathPlannerPath.fromPathFile(getNearestTrenchPath());

            return AutoBuilder.pathfindThenFollowPath(pathPlannerPath, TeleopConstants.CONSTRAINTS);
        } catch (Exception e) {
            return null;
        }
    }

    /**
     * 
     */
    private String getNearestTrenchPath() {
        Pose2d robotPose = robotPoseSupplier.get();
        Pose2d leftTrenchPose = AllianceFlipUtil.apply(FieldConstants.TRENCH_POSES[0]);
        Pose2d rightTrenchPose = AllianceFlipUtil.apply(FieldConstants.TRENCH_POSES[1]);

        double leftTrenchDistance = robotPose.getTranslation().getDistance(leftTrenchPose.getTranslation());
        double rightTrenchDistance = robotPose.getTranslation().getDistance(rightTrenchPose.getTranslation());

        if (AllianceFlipUtil.shouldFlip()) {
            return leftTrenchDistance < rightTrenchDistance ? "Drive Right Trench Path" : "Drive Left Trench Path";
        } else {
            return leftTrenchDistance < rightTrenchDistance ? "Drive Left Trench Path" : "Drive Right Trench Path";
        }
    }
}
