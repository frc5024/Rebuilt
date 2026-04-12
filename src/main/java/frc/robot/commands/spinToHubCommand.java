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

        // Calculate the turret's position on the field
        Translation2d turretFieldPos = robotPose.getTranslation().plus(turretOffset.rotateBy(robotPose.getRotation()));

        // Calculate the vector from the turret to the target
        Translation2d turretToTarget = targetPose.getTranslation().minus(turretFieldPos);

        // Default (no lead) target is the hub center
        Translation2d compensatedTarget = targetPose.getTranslation();

        // Debug/diagnostic variables (wider scope so logs are safe)
        double projectileSpeed = Double.NaN;
        double tof = Double.NaN;
        ChassisSpeeds chassis = null;
        double turretVx = Double.NaN;
        double turretVy = Double.NaN;
        double predX = Double.NaN;
        double predY = Double.NaN;
        double chosenVxGlobal = Double.NaN;
        double chosenVyGlobal = Double.NaN;
        boolean haveChosenMuzzle = false;

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
                    double distance = turretToTarget.getNorm();

                    // Estimate TOF simply as distance / muzzle speed
                    tof = distance / projectileSpeed;
                    if (Double.isFinite(tof) && tof > 0.0) {
                        // Compute required projectile velocity (field frame) using the
                        // physically-correct relation: muzzleVel = (target - turretPos)/tof - turretVel
                        // Also compute the alternate sign candidate (historical confusion)
                        double reqVx_sub = turretToTarget.getX() / tof - turretVx;
                        double reqVy_sub = turretToTarget.getY() / tof - turretVy;

                        double reqVx_add = turretToTarget.getX() / tof + turretVx;
                        double reqVy_add = turretToTarget.getY() / tof + turretVy;

                        double mag_sub = Math.hypot(reqVx_sub, reqVy_sub);
                        double mag_add = Math.hypot(reqVx_add, reqVy_add);

                        // Compare which candidate's magnitude is closer to the measured projectile
                        // speed
                        double err_sub = Math.abs(mag_sub - projectileSpeed);
                        double err_add = Math.abs(mag_add - projectileSpeed);

                        // Hysteresis: prefer previous choice unless there's a clear improvement
                        boolean chooseSub;
                        if (Math.abs(err_sub - err_add) < CANDIDATE_SWITCH_THRESHOLD) {
                            chooseSub = lastUsedSub;
                        } else {
                            chooseSub = err_sub <= err_add;
                        }

                        double chosenVx = chooseSub ? reqVx_sub : reqVx_add;
                        double chosenVy = chooseSub ? reqVy_sub : reqVy_add;
                        boolean usedSub = chooseSub;
                        lastUsedSub = chooseSub;

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

                        // Log which candidate was chosen and the errors
                        Logger.recordOutput("Turret/Lead/ChosenIsSub", usedSub);
                        Logger.recordOutput("Turret/Lead/MagSub", mag_sub);
                        Logger.recordOutput("Turret/Lead/MagAdd", mag_add);
                        Logger.recordOutput("Turret/Lead/ErrSub", err_sub);
                        Logger.recordOutput("Turret/Lead/ErrAdd", err_add);
                    }
                }
            }
        } catch (Exception e) {
            // If anything goes wrong, fall back to aiming at hub center
        }

        // Debug logging to help tune/verify sign and values
        Logger.recordOutput("Turret/Lead/Tof", tof);
        Logger.recordOutput("Turret/Lead/ProjectileSpeed", projectileSpeed);
        Logger.recordOutput("Turret/Lead/ChassisVx", chassis == null ? Double.NaN : chassis.vxMetersPerSecond);
        Logger.recordOutput("Turret/Lead/ChassisVy", chassis == null ? Double.NaN : chassis.vyMetersPerSecond);
        Logger.recordOutput("Turret/Lead/TurretVx", turretVx);
        Logger.recordOutput("Turret/Lead/TurretVy", turretVy);
        Logger.recordOutput("Turret/Lead/PredX", predX);
        Logger.recordOutput("Turret/Lead/PredY", predY);

        // Compute desired turret angle. If we computed a muzzle velocity that fits the
        // shooter speed, use its bearing directly (more robust). Otherwise fall back to
        // aiming at the compensated target point.
        Rotation2d turretAngleVar = new Rotation2d();
        Pose2d turretPoseVar = new Pose2d(turretFieldPos, turretAngleVar);

        if (haveChosenMuzzle && Double.isFinite(chosenVxGlobal) && Double.isFinite(chosenVyGlobal)) {
            // Use the chosen muzzle vector directly (no sign flip)
            double muzzleAngle = Math.atan2(chosenVyGlobal, chosenVxGlobal); // field-frame angle of muzzle velocity
            Rotation2d muzzleRot = new Rotation2d(muzzleAngle);
            // Robot is CCW+ / Turret is CW+
            turretAngleVar = muzzleRot.minus(robotPose.getRotation()).unaryMinus();
            turretPoseVar = new Pose2d(turretFieldPos, muzzleRot);
            Logger.recordOutput("Turret/UsingMuzzleVector", true);
            Logger.recordOutput("Turret/MuzzleAngleRad", muzzleAngle);
        } else {
            // The turret's desired pose (aiming at compensated target)
            Translation2d turretToComp = compensatedTarget.minus(turretFieldPos);
            turretPoseVar = new Pose2d(turretFieldPos, turretToComp.getAngle());

            // Calculate the desired turret angle
            // Robot is CCW+ / Turret is CW+
            turretAngleVar = turretPoseVar.getRotation().minus(robotPose.getRotation()).unaryMinus();
            Logger.recordOutput("Turret/UsingMuzzleVector", false);
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
