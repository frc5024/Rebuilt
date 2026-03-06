package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
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
        // System.out.println("setting turret angle");
        // turretSubsystem.resetPID(); // turretSubsystem.zeroEncoder();
        // turretSubsystem.setTargetAngle(targetAngle);
        // turretSubsystem.enablePID();
    }

    @Override
    public void execute() {

        Pose2d targetPose = getTarget();
        double targetAngle = Math.atan2(targetPose.getY() - robotPoseSupplier.get().getY(),
                targetPose.getX() - robotPoseSupplier.get().getX());

        turretSubsystem.setTargetAngle(Units.radiansToDegrees(targetAngle));

        // System.out.println(turretSubsystem.getTurretAngle());
        // turretSubsystem.updateTurretAngle();

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

    private Pose2d getTarget() {
        Pose2d robotPose = robotPoseSupplier.get();

        // boolean isRedAlliance =
        // DriverStation.getAlliance().equals(DriverStation.Alliance.Red);

        double robotX = robotPose.getX();
        double robotY = robotPose.getY();

        Pose2d hubPose = GameUtil.getHubPose();
        double hubX = hubPose.getX();
        double hubY = hubPose.getY();

        return hubPose;
    }

}
