package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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
    private final Supplier<ChassisSpeeds> robotVelocitySupplier;
    private final DoubleSupplier ballVelocitySupplier;

    /**
     * 
     */
    public LockTurretOnTarget(TurretSubsystem turretSubsystem, Supplier<Pose2d> robotPoseSupplier,
            Supplier<ChassisSpeeds> robotVelocitySupplier, DoubleSupplier ballVelocitySupplier) {
        this.turretSubsystem = turretSubsystem;
        this.robotPoseSupplier = robotPoseSupplier;
        this.robotVelocitySupplier = robotVelocitySupplier;
        this.ballVelocitySupplier = ballVelocitySupplier;

        addRequirements(turretSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        Pose2d robotPose = robotPoseSupplier.get();
        Pose2d targetPose = GameUtil.getTargetPose(robotPose);
        ChassisSpeeds robotVelocity = robotVelocitySupplier.get();
        double ballVelocity = ballVelocitySupplier.getAsDouble();

        Translation2d turretOffset = new Translation2d(TurretConstants.OFFSET_X, TurretConstants.OFFSET_Y);
        Translation2d turretFieldPos = robotPose.transformBy(new Transform2d(turretOffset, new Rotation2d()))
                .getTranslation();

        double distanceToTarget = turretFieldPos.getDistance(targetPose.getTranslation());
        double timeOfFlight = ballVelocity > 0.0 ? distanceToTarget / ballVelocity : 0;

        double virtualX = targetPose.getX() - (robotVelocity.vxMetersPerSecond * timeOfFlight);
        double virtualY = targetPose.getY() - (robotVelocity.vxMetersPerSecond * timeOfFlight);
        Translation2d virtualTarget = new Translation2d(virtualX, virtualY);

        Rotation2d fieldAngle = virtualTarget.minus(turretFieldPos).getAngle();
        Rotation2d turretAngle = fieldAngle.minus(robotPose.getRotation());

        turretSubsystem.setAngle(turretAngle.getDegrees());

        Logger.recordOutput("Turret/LockTurretOnTarget/DistanceToTarget", distanceToTarget);
        Logger.recordOutput("Turret/LockTurretOnTarget/RobotPoseDeg", robotPose.getRotation().getDegrees());
        Logger.recordOutput("Turret/LockTurretOnTarget/TargetPose", virtualTarget);
        Logger.recordOutput("Turret/LockTurretOnTarget/TargetPoseDeg", virtualTarget.getAngle().getDegrees());
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
