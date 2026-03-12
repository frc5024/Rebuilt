package frc.robot.commands;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.util.GameUtil;

/**
 * 
 */
public class spinToHubCommand extends Command {
    // Subsystems
    private final TurretSubsystem turretSubsystem;
    private final Supplier<Pose2d> robotPoseSupplier;
    private final Supplier<ChassisSpeeds> chassisSpeedSupplier;

    /**
     * 
     */
    public spinToHubCommand(TurretSubsystem turretSubsystem, Supplier<Pose2d> robotPoseSupplier,
            Supplier<ChassisSpeeds> chassisSpeedSupplier) {
        this.turretSubsystem = turretSubsystem;
        this.robotPoseSupplier = robotPoseSupplier;
        this.chassisSpeedSupplier = chassisSpeedSupplier;

        addRequirements(turretSubsystem);
    }

    @Override
    public void initialize() {
        turretSubsystem.enablePID();
    }

    @Override
    public void execute() {
        Pose2d robotPose = robotPoseSupplier.get();

        double robotX = robotPose.getX();
        double robotY = robotPose.getY();

        Pose2d targetPose = getTargetPose(robotPose);
        double targetX = targetPose.getX();
        double targetY = targetPose.getY();

        Rotation2d angleToHub = Rotation2d.fromRadians(Math.atan2(targetY - robotY, targetX - robotX))
                .rotateBy(Rotation2d.k180deg);

        Rotation2d fieldAngle = targetPose.getTranslation().getAngle();
        // Rotation2d turretAngle =
        // fieldAngle.minus(robotPose.getRotation().plus(Rotation2d.fromDegrees(180)));
        // turretSubsystem.setTargetAngle(-turretAngle.getDegrees());

        // turretAngle = turretAngle.rotateBy(angleToHub.unaryMinus());
        Rotation2d robotRotation = robotPose.getRotation().rotateBy(Rotation2d.k180deg);

        Rotation2d turretAngle = robotRotation.plus(angleToHub.unaryMinus());

        turretSubsystem.setAngle(turretAngle.getDegrees());

        if (RobotConstants.TUNING_MODE) {
            ShuffleboardTab tab = Shuffleboard.getTab("spinToHub");

            SmartDashboard.putNumber("fieldAngle", fieldAngle.getDegrees());
            SmartDashboard.putNumber("turretAngle", turretAngle.getDegrees());
            SmartDashboard.putNumber("robotRotation", robotRotation.getDegrees());
            SmartDashboard.putNumber("angleToHub", angleToHub.getDegrees());
        }

        Logger.recordOutput("Turret/Active Command", this.getName());
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        turretSubsystem.disablePID();

        Logger.recordOutput("Turret/Active Command", "");
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
}
