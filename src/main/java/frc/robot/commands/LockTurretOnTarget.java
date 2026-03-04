package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants;
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
        Pose2d targetPose = getTargetPose(robotPose);
        ChassisSpeeds robotVelocity = robotVelocitySupplier.get();
        double ballVelocity = ballVelocitySupplier.getAsDouble();

        double distanceToTarget = robotPose.getTranslation().getDistance(targetPose.getTranslation());
        double timeOfFlight = distanceToTarget / ballVelocity;

        double virtualX = targetPose.getX() - (robotVelocity.vxMetersPerSecond * timeOfFlight);
        double virtualY = targetPose.getY() - (robotVelocity.vxMetersPerSecond * timeOfFlight);
        Translation2d virtualTarget = new Translation2d(virtualX, virtualY);

        Rotation2d fieldAngle = virtualTarget.minus(robotPose.getTranslation()).getAngle();
        Rotation2d turretAngle = fieldAngle.minus(robotPose.getRotation());

        turretSubsystem.setAngle(turretAngle.getDegrees());

        Logger.recordOutput("Commands/Active Command", this.getName());
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        Logger.recordOutput("Commands/Active Command", "");
    }

    /**
     * 
     */
    public Pose2d getTargetPose(Pose2d robotPose) {
        boolean isRedAlliance = DriverStation.getAlliance().get() == Alliance.Red;
        boolean isAboveMidLine = GameUtil.isAboveMidLine(robotPose);

        if (GameUtil.inAllianceZone(robotPose)) {
            return GameUtil.getHubPose();
        } else {
            return FieldConstants.MULE_POSES[isRedAlliance ? 1 : 0][isAboveMidLine ? 1 : 0];
        }
    }
}
