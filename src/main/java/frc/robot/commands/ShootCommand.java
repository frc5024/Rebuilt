package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FeederConstants;
import frc.robot.Constants.HopperConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.SwerveDriveSubsystem;
import frc.robot.util.GameUtil;

/**
 * 
 */
public class ShootCommand extends Command {
    // Subsystems
    private final SwerveDriveSubsystem swerveDriveSubsystem;
    private final ShooterSubsystem shooterSubsystem;
    private final HopperSubsystem hopperSubsystem;
    private final FeederSubsystem feederSubsystem;
    private final Supplier<Pose2d> robotPoseSupplier;

    // Variables
    private Pose2d targetPose;

    /**
     * 
     */
    public ShootCommand(SwerveDriveSubsystem swerveDriveSubsystem, ShooterSubsystem shooterSubsystem,
            HopperSubsystem hopperSubsystem, FeederSubsystem feederSubsystem, Supplier<Pose2d> robotPoseSupplier) {
        this.swerveDriveSubsystem = swerveDriveSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.hopperSubsystem = hopperSubsystem;
        this.feederSubsystem = feederSubsystem;
        this.robotPoseSupplier = robotPoseSupplier;

        addRequirements(shooterSubsystem, hopperSubsystem, feederSubsystem);
    }

    @Override
    public void initialize() {
        // Only allow slow driving
        swerveDriveSubsystem.setSlowMode(true);
    }

    @Override
    public void execute() {
        // get the target we want to shoot at
        Pose2d robotPose = robotPoseSupplier.get();
        targetPose = GameUtil.getTargetPose(robotPose);

        // SpinToHub and ShooterSubsystem handle setting shooter RPM

        // spin up hooper and feeder if shooter is ready
        if (shooterSubsystem.isAtSetpoint()) {
            feederSubsystem.setVelocity(FeederConstants.RPM);
            hopperSubsystem.setVelocity(HopperConstants.RPM);
        }
    }

    @Override
    public void end(boolean interrupted) {
        feederSubsystem.setVelocity(0.0);
        hopperSubsystem.setVelocity(0.0);
        shooterSubsystem.setVelocity(ShooterConstants.IDLE_SPEED_RPM);
        swerveDriveSubsystem.setSlowMode(false);
    }

    @Override
    public boolean isFinished() {
        // run until the trigger is released
        return false;
    }
}
