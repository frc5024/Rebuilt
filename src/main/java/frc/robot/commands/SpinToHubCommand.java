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
public class SpinToHubCommand extends Command {
    // Subsystems
    private final TurretSubsystem turretSubsystem;
    private final Supplier<Pose2d> robotPoseSupplier;
    private final Supplier<ChassisSpeeds> robotVelocitySupplier;
    private final DoubleSupplier ballVelocitySupplier;
    private final DistanceConsumer distanceConsumer;

    /**
     * 
     */
    public SpinToHubCommand(TurretSubsystem turretSubsystem, Supplier<Pose2d> robotPoseSupplier,
            Supplier<ChassisSpeeds> robotVelocitySupplier, DoubleSupplier ballVelocitySupplier,
            DistanceConsumer distanceConsumer) {
        this.turretSubsystem = turretSubsystem;
        this.robotPoseSupplier = robotPoseSupplier;
        this.robotVelocitySupplier = robotVelocitySupplier;
        this.ballVelocitySupplier = ballVelocitySupplier;
        this.distanceConsumer = distanceConsumer;

        addRequirements(turretSubsystem);
    }

    @Override
    public void initialize() {
        turretSubsystem.enablePID();
    }

    @Override
    public void execute() {
        Pose2d robotPose = robotPoseSupplier.get();
        Pose2d targetPose = getTargetPose(robotPose);
        ChassisSpeeds robotVelocity = robotVelocitySupplier.get();
        double ballVelocity = ballVelocitySupplier.getAsDouble();

        // Turret's offset from the robot's center and turret's position on the field
        Translation2d turretOffset = new Translation2d(-0.15, 0.1778);
        Translation2d turretFieldPos = robotPose.getTranslation().plus(turretOffset.rotateBy(robotPose.getRotation()));

        double distanceToTarget = turretFieldPos.getDistance(targetPose.getTranslation());
        double timeOfFlight = ballVelocity > 0.0 ? distanceToTarget / ballVelocity : 0;

        // define a virtual target based on robot velocity vectors
        double virtualX = robotVelocity.vxMetersPerSecond * timeOfFlight;
        double virtualY = robotVelocity.vyMetersPerSecond * timeOfFlight;
        Translation2d virtualTranslation = new Translation2d(virtualX, virtualY);
        Translation2d virtualTarget = targetPose.getTranslation().minus(virtualTranslation);

        // Calculate the vector from the turret to the target
        Translation2d turretToVirtualTarget = virtualTarget.minus(turretFieldPos);
        // double shootDistance = turretToVirtualTarget.getDistance(virtualTarget);
        double shootDistance = turretFieldPos.getDistance(virtualTarget);
        distanceConsumer.accept(shootDistance);

        // Calculate the desired turret angle Robot is CCW+ / Turret is CW+
        Rotation2d turretFieldRotation = turretToVirtualTarget.getAngle();
        Rotation2d turretAngle = turretFieldRotation.minus(robotPose.getRotation()).unaryMinus();

        turretSubsystem.setAngle(turretAngle.getDegrees());

        Logger.recordOutput("Turret/DistanceToTarget", distanceToTarget);
        Logger.recordOutput("Turret/ShootDistance", shootDistance);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        turretSubsystem.disablePID();
    }

    /**
     * 
     */
    private Pose2d getTargetPose(Pose2d robotPose) {
        boolean isRedAlliance = DriverStation.getAlliance().get() == Alliance.Red;
        boolean isAboveMidLine = GameUtil.isAboveMidLine(robotPose);

        if (GameUtil.inAllianceZone(robotPose)) {
            return GameUtil.getHubPose();
        } else {
            return FieldConstants.MULE_POSES[isRedAlliance ? 1 : 0][isAboveMidLine ? 1 : 0];
        }
    }

    @FunctionalInterface
    public static interface DistanceConsumer {
        public void accept(double distanceToTarget);
    }
}
