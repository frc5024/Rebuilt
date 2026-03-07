package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.util.GameUtil;

public class spinToHubCommand extends Command {

    private final TurretSubsystem turretSubsystem;
    private final Supplier<Pose2d> robotPoseSupplier;
    private final Supplier<ChassisSpeeds> chassisSpeedSupplier;

    // CommandXboxController operator = RobotContainer.operator;

    public spinToHubCommand(TurretSubsystem turretSubsystem, Supplier<Pose2d> robotPoseSupplier,
            Supplier<ChassisSpeeds> chassisSpeedSupplier) {
        this.turretSubsystem = turretSubsystem;
        this.robotPoseSupplier = robotPoseSupplier;
        this.chassisSpeedSupplier = chassisSpeedSupplier;
        addRequirements(turretSubsystem);
    }

    @Override
    public void initialize() {
        turretSubsystem.zeroEncoder();
        turretSubsystem.enablePID();
    }

    @Override
    public void execute() {

        Pose2d robotPose = robotPoseSupplier.get();
        Pose2d targetPose = getTargetPose(robotPose);

        Rotation2d fieldAngle = targetPose.getTranslation().getAngle();
        Rotation2d turretAngle = fieldAngle.minus(robotPose.getRotation());
        turretSubsystem.setTargetAngle(turretAngle.getDegrees());

    }

    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        turretSubsystem.disablePID();
        // Optionally stop motor explicitly
        turretSubsystem.setIdle();
    }

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
