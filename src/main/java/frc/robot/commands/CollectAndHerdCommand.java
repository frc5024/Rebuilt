package frc.robot.commands;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.util.AllianceFlipUtil;

/**
 * 
 */
public class CollectAndHerdCommand extends Command {
    // Subsystems
    private final IntakeSubsystem intakeSubsystem;
    private final Supplier<Pose2d> robotPoseSupplier;

    // Event triggers
    private final EventTrigger extendAndIntakeTrigger;
    private final EventTrigger retractTrigger;

    // Variables
    private final PathConstraints CONSTRAINTS = new PathConstraints(2.0, 4.0, Units.degreesToRadians(540),
            Units.degreesToRadians(720));
    private Command followPathCommand;

    /**
     * 
     */
    public CollectAndHerdCommand(IntakeSubsystem intakeSubsystem, Supplier<Pose2d> robotPoseSupplier) {
        this.intakeSubsystem = intakeSubsystem;
        this.robotPoseSupplier = robotPoseSupplier;

        this.extendAndIntakeTrigger = new EventTrigger("ExtendAndIntake");
        this.retractTrigger = new EventTrigger("Retract");

        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        followPathCommand = getFollowPathCommand();

        if (followPathCommand != null) {
            followPathCommand.initialize();

            // define the event triggers
            extendAndIntakeTrigger.onTrue(
                    Commands.sequence(
                            Commands.runOnce(() -> intakeSubsystem.intakeRoller()),
                            Commands.runOnce(() -> intakeSubsystem.extendArm())));

            retractTrigger.onTrue(
                    Commands.sequence(
                            Commands.runOnce(() -> intakeSubsystem.stopRoller()),
                            Commands.runOnce(() -> intakeSubsystem.retractArm())));

            Logger.recordOutput("Commands/Active Command", this.getName());
        }
    }

    @Override
    public void execute() {
        if (followPathCommand != null) {
            followPathCommand.execute();

            Logger.recordOutput("PathPlanner/Events/ExtendAndIntakeMarker", extendAndIntakeTrigger.getAsBoolean());
            Logger.recordOutput("PathPlanner/Events/RetractMarker", retractTrigger.getAsBoolean());
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
            PathPlannerPath pathPlannerPath = PathPlannerPath.fromPathFile(getNearestWallPath());

            return AutoBuilder.pathfindThenFollowPath(pathPlannerPath, CONSTRAINTS);
        } catch (Exception e) {
            return null;
        }
    }

    /**
     * 
     */
    private String getNearestWallPath() {
        Pose2d robotPose = robotPoseSupplier.get();
        Pose2d leftTrenchPose = AllianceFlipUtil.apply(FieldConstants.TRENCH_POSES[0]);
        Pose2d rightTrenchPose = AllianceFlipUtil.apply(FieldConstants.TRENCH_POSES[1]);

        double leftTrenchDistance = robotPose.getTranslation().getDistance(leftTrenchPose.getTranslation());
        double rightTrenchDistance = robotPose.getTranslation().getDistance(rightTrenchPose.getTranslation());

        if (AllianceFlipUtil.shouldFlip()) {
            return leftTrenchDistance < rightTrenchDistance ? "Collect And Herd Left Path"
                    : "Collect And Herd Right Path";
        } else {
            return leftTrenchDistance < rightTrenchDistance ? "Collect And Herd Right Path"
                    : "Collect And Herd Left Path";
        }
    }
}
