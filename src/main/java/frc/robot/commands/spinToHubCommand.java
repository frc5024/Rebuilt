package frc.robot.commands;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.util.GameUtil;

/**
 * 
 */
public class spinToHubCommand extends Command {
    // Subsystems
    private final TurretSubsystem turretSubsystem;
    private final ShooterSubsystem shooterSubsystem;
    private final Supplier<Pose2d> robotPoseSupplier;
    private final Supplier<ChassisSpeeds> chassisSpeedSupplier;
    // Smoothing / hysteresis state to avoid sudden jumps when candidate switches
    private double prevPredX = Double.NaN;
    private double prevPredY = Double.NaN;
    private double prevAngleDeg = Double.NaN;
    private boolean lastUsedSub = true;
    private static final double POS_SMOOTH_ALPHA = 0.2; // 0..1 (higher = more responsive)
    private static final double ANGLE_SMOOTH_ALPHA = 0.3;
    private static final double CANDIDATE_SWITCH_THRESHOLD = 0.5; // m/s hysteresis on projectile speed error

    /**
     * 
     */
    public spinToHubCommand(TurretSubsystem turretSubsystem, ShooterSubsystem shooterSubsystem,
            Supplier<Pose2d> robotPoseSupplier, Supplier<ChassisSpeeds> chassisSpeedSupplier) {
        this.turretSubsystem = turretSubsystem;
        this.shooterSubsystem = shooterSubsystem;
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

        double rotatedOffsetX = turretOffset.getX() * Math.cos(robotPose.getRotation().getRadians())
                - turretOffset.getY() * Math.sin(robotPose.getRotation().getRadians());
        double rotatedOffsetY = turretOffset.getX() * Math.sin(robotPose.getRotation().getRadians())
                + turretOffset.getY() * Math.cos(robotPose.getRotation().getRadians());

        // Calculate turret center position
        double turretX = robotPose.getX() + rotatedOffsetX;
        double turretY = robotPose.getY() + rotatedOffsetY;

        // Calculate the turret's position on the field
        Translation2d turretFieldPos = new Translation2d(turretX, turretY);

        // Calculate the vector from the turret to the target
        Translation2d turretToTarget = targetPose.getTranslation().minus(turretFieldPos);

        // Default (no lead) target is the hub center
        Translation2d compensatedTarget = targetPose.getTranslation();

        // Debug/diagnostic variables (wider scope so logs are safe)
        double projectileSpeed = Double.NaN;
        ChassisSpeeds chassis = null;
        double turretVx = Double.NaN;
        double turretVy = Double.NaN;
        double predX = Double.NaN;
        double predY = Double.NaN;
        double chosenVxGlobal = Double.NaN;
        double chosenVyGlobal = Double.NaN;
        double lastTof = Double.NaN;
        boolean haveChosenMuzzle = true;

        try {
            // If we have a shooter, compute a simple lead based on current chassis
            // velocity and muzzle speed
            if (shooterSubsystem != null && chassisSpeedSupplier != null) {
                projectileSpeed = shooterSubsystem.getTangentialVelocity(); // m/s

                // Only try to lead when projectile speed is reasonable
                if (!Double.isNaN(projectileSpeed) && projectileSpeed > 0.1) {
                    chassis = chassisSpeedSupplier.get();

                    double heading = robotPose.getRotation().getRadians();
                    double cosH = Math.cos(heading);
                    double sinH = Math.sin(heading);

                    // Convert chassis speeds (robot frame) to field frame
                    double vxField = chassis.vxMetersPerSecond * cosH - chassis.vyMetersPerSecond * sinH;
                    double vyField = chassis.vxMetersPerSecond * sinH + chassis.vyMetersPerSecond * cosH;
                    double omega = chassis.omegaRadiansPerSecond;

                    // Turret offset in field coords
                    double offX = turretOffset.getX() * cosH - turretOffset.getY() * sinH;
                    double offY = turretOffset.getX() * sinH + turretOffset.getY() * cosH;

                    // Turret linear velocity (field) includes rotational component
                    turretVx = vxField + (-offY) * omega;
                    turretVy = vyField + offX * omega;

                    // Distance from turret to target
                    double distance = (robotPose.getTranslation().getDistance(targetPose.getTranslation()));
                    double tof = Constants.shooterConstants.distanceToTOFMap.get(distance);

                    // Estimate TOF simply as distance / muzzle speed
                    if (Double.isFinite(tof) && tof > 0.0) {
                        // Compute required projectile velocity (field frame) using the
                        // physically-correct relation: muzzleVel = (target - turretPos)/tof - turretVel
                        // Also compute the alternate sign candidate (historical confusion)
                        // Use the physically-correct muzzle relation: muzzleVel = (target -
                        // turretPos)/tof - turretVel
                        // (always subtract turret linear velocity). This avoids sign confusion where
                        // the
                        // alternate "add" candidate produces lead in the wrong direction.
                        double reqVx_sub = turretToTarget.getX() / tof - turretVx;
                        double reqVy_sub = turretToTarget.getY() / tof - turretVy;

                        double chosenVx = reqVx_sub;
                        double chosenVy = reqVy_sub;
                        boolean usedSub = true;
                        lastUsedSub = true;

                        // Construct a compensated target point consistent with the chosen muzzle
                        // velocity
                        predX = turretFieldPos.getX() + chosenVx * tof;
                        predY = turretFieldPos.getY() + chosenVy * tof;

                        // Smooth predicted point to avoid sudden jumps when switching
                        if (Double.isFinite(prevPredX)) {
                            predX = POS_SMOOTH_ALPHA * predX + (1.0 - POS_SMOOTH_ALPHA) * prevPredX;
                            predY = POS_SMOOTH_ALPHA * predY + (1.0 - POS_SMOOTH_ALPHA) * prevPredY;
                        }
                        prevPredX = predX;
                        prevPredY = predY;

                        compensatedTarget = new Translation2d(predX, predY);

                        // Save chosen muzzle vector so we can compute turret bearing directly
                        chosenVxGlobal = chosenVx;
                        chosenVyGlobal = chosenVy;
                        haveChosenMuzzle = true;
                        lastTof = tof;

                        // Log chosen candidate (we always use the subtract candidate)
                        Logger.recordOutput("Turret/Lead/ChosenIsSub", usedSub);
                        Logger.recordOutput("Turret/Lead/MagSub", Math.hypot(chosenVx, chosenVy));

                        Logger.recordOutput("Turret/Lead/ProjectileSpeed", projectileSpeed);
                        Logger.recordOutput("Turret/Lead/ChassisVx",
                                chassis == null ? Double.NaN : chassis.vxMetersPerSecond);
                        Logger.recordOutput("Turret/Lead/ChassisVy",
                                chassis == null ? Double.NaN : chassis.vyMetersPerSecond);
                        Logger.recordOutput("Turret/Lead/TurretVx", turretVx);
                        Logger.recordOutput("Turret/Lead/TurretVy", turretVy);
                        Logger.recordOutput("Turret/Lead/PredX", predX);
                        Logger.recordOutput("Turret/Lead/PredY", predY);

                        // Additional diagnostics: shift applied (turretVel * tof) and vector from
                        // raw target -> predicted target
                        Logger.recordOutput("Turret/Lead/ShiftX", turretVx * tof);
                        Logger.recordOutput("Turret/Lead/ShiftY", turretVy * tof);
                        double targetToPredX = predX - targetPose.getTranslation().getX();
                        double targetToPredY = predY - targetPose.getTranslation().getY();
                        Logger.recordOutput("Turret/Lead/TargetToPredX", targetToPredX);
                        Logger.recordOutput("Turret/Lead/TargetToPredY", targetToPredY);
                    }
                }
            }
        } catch (Exception e) {
            // If anything goes wrong, fall back to aiming at hub center
        }

        // Debug logging to help tune/verify sign and values

        // Compute desired turret angle. If we computed a muzzle velocity that fits the
        // shooter speed, use its bearing directly (more robust). Otherwise fall back to
        // aiming at the compensated target point.
        Rotation2d turretAngleVar = new Rotation2d();
        Pose2d turretPoseVar = new Pose2d(turretFieldPos, turretAngleVar);

        if (haveChosenMuzzle) {
            // Aim at the compensated/predicted target when available
            Translation2d turretToComp = compensatedTarget.minus(turretFieldPos);
            turretPoseVar = new Pose2d(turretFieldPos, turretToComp.getAngle());

            // Calculate the desired turret angle (Robot CCW+, Turret CW+)
            turretAngleVar = turretPoseVar.getRotation().minus(robotPose.getRotation()).unaryMinus();
            Logger.recordOutput("Turret/UsingCompensatedTarget", true);
        } else {
            // Fallback: aim at hub center (compensatedTarget == hub center)
            Translation2d turretToComp = compensatedTarget.minus(turretFieldPos);
            turretPoseVar = new Pose2d(turretFieldPos, turretToComp.getAngle());
            turretAngleVar = turretPoseVar.getRotation().minus(robotPose.getRotation()).unaryMinus();
            Logger.recordOutput("Turret/UsingCompensatedTarget", false);
        }

        // Smooth the commanded turret angle to remove abrupt jumps
        double newAngleDeg = turretAngleVar.getDegrees();
        if (Double.isFinite(prevAngleDeg)) {
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
        } else {
            prevAngleDeg = newAngleDeg;
        }
        turretSubsystem.setAngle(turretAngleVar.getDegrees());
        Logger.recordOutput("Turret/TargetAngleRad", turretAngleVar);
        Logger.recordOutput("Turret/TargetAngleDeg", turretAngleVar.getDegrees());
        Logger.recordOutput("Turret/TargetPose", new Pose3d(targetPose));
        Logger.recordOutput("Turret/CompensatedTarget",
                new Pose3d(new Pose2d(compensatedTarget, turretPoseVar.getRotation())));
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
