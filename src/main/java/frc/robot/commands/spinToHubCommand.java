package frc.robot.commands;

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
        Pose2d targetPose = getTargetPose(robotPose);

        // Turret's offset from the robot's center
        Translation2d turretOffset = new Translation2d(-0.15, 0.1778);

        // Calculate the turret's position on the field
        Translation2d turretFieldPos = robotPose.getTranslation().plus(turretOffset.rotateBy(robotPose.getRotation()));

        // Calculate the vector from the turret to the target
        Translation2d turretToTarget = targetPose.getTranslation().minus(turretFieldPos);

        // The turret's desired pose
        Pose2d turretPose = new Pose2d(turretFieldPos, turretToTarget.getAngle());

        // Calculate the desired turret angle
        // Robot is CCW+ / Turret is CW+
        turretAngleVar = turretPoseVar.getRotation().minus(robotPose.getRotation()).unaryMinus();
        Logger.recordOutput("Turret/UsingMuzzleVector", false);
    }

    // Smooth the commanded turret angle to remove abrupt jumps
    double newAngleDeg = turretAngleVar.getDegrees();if(Double.isFinite(prevAngleDeg))
    {
        double delta = newAngleDeg - prevAngleDeg;
        // Normalize to [-180, 180]
        while (delta > 180.0) {
            delta -= 360.0;
        }
        while (delta < -180.0) {
            delta += 360.0;
        }
        double smoothedDeg = newAngleDeg + ANGLE_SMOOTH_ALPHA * delta;
        turretAngleVar = Rotation2d.fromDegrees(smoothedDeg);
        prevAngleDeg = smoothedDeg;
    }else
    {
        prevAngleDeg = newAngleDeg;
    }turretSubsystem.setAngle(turretAngleVar.getDegrees());Logger.recordOutput("Turret/TargetAngleRad",turretAngleVar);Logger.recordOutput("Turret/TargetAngleDeg",turretAngleVar.getDegrees());Logger.recordOutput("Turret/TargetPose",new Pose3d(targetPose));
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
