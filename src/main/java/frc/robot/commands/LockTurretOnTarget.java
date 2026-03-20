package frc.robot.commands;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.util.GameUtil;

/**
 * 
 */
public class LockTurretOnTarget extends Command {
    // Subsystems
    private final TurretSubsystem turretSubsystem;
    private final Supplier<Pose2d> robotPoseSupplier;

    /**
     * 
     */
    public LockTurretOnTarget(TurretSubsystem turretSubsystem, Supplier<Pose2d> robotPoseSupplier) {
        this.turretSubsystem = turretSubsystem;
        this.robotPoseSupplier = robotPoseSupplier;

        addRequirements(turretSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        Pose2d robotPose = robotPoseSupplier.get();
        Pose2d targetPose = GameUtil.getTargetPose(robotPose);

        Translation2d turretOffset = new Translation2d(TurretConstants.OFFSET_X, TurretConstants.OFFSET_Y);
        Translation2d turretFieldPos = robotPose.transformBy(new Transform2d(turretOffset, new Rotation2d()))
                .getTranslation();

        double distanceToTarget = turretFieldPos.getDistance(targetPose.getTranslation());

        Rotation2d fieldAngle = targetPose.getTranslation().minus(turretFieldPos).getAngle();
        Rotation2d turretAngle = fieldAngle.minus(robotPose.getRotation());

        turretSubsystem.setAngle(turretAngle.getDegrees());

        Logger.recordOutput("Turret/LockTurretOnTarget/DistanceToTarget", distanceToTarget);
        Logger.recordOutput("Turret/LockTurretOnTarget/RobotPoseDeg", robotPose.getRotation().getDegrees());
        Logger.recordOutput("Turret/LockTurretOnTarget/TargetPose", targetPose);
        Logger.recordOutput("Turret/LockTurretOnTarget/TargetPoseDeg", targetPose.getRotation().getDegrees());
        Logger.recordOutput("Turret/LockTurretOnTarget/TargetSetPointDeg", turretAngle.getDegrees());
        Logger.recordOutput("Turret/Active Command", this.getName());
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        Logger.recordOutput("Turret/Active Command", "");
    }
}
