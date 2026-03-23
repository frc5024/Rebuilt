package frc.robot.commands;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
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

        // Turret's offset from the robot's center
        Translation2d turretOffset = new Translation2d(-0.254, 0.1778);

        // Calculate the turret's position on the field
        Translation2d turretFieldPos = robotPose.getTranslation().plus(turretOffset.rotateBy(robotPose.getRotation()));
        double distanceToTarget = turretFieldPos.getDistance(targetPose.getTranslation());

        // Calculate the vector from the turret to the target
        Translation2d turretToTarget = targetPose.getTranslation().minus(turretFieldPos);

        // The turret's desired pose
        Pose2d turretPose = new Pose2d(turretFieldPos, turretToTarget.getAngle());

        // Calculate the desired turret angle
        // Robot is CCW+ / Turret is CW+
        Rotation2d turretAngle = turretPose.getRotation().minus(robotPose.getRotation()).unaryMinus();

        turretSubsystem.setAngle(turretAngle.getDegrees());

        Logger.recordOutput("Turret/TargetAngleRad", turretAngle);
        Logger.recordOutput("Turret/TargetAngleDeg", turretAngle.getDegrees());
        Logger.recordOutput("Turret/TargetPose", new Pose3d(targetPose));
        Logger.recordOutput("Turret/FieldPose", new Pose3d(turretPose));
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
