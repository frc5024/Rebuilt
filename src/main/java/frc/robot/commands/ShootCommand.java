package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FeederConstants;
import frc.robot.Constants.HopperConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.SwerveDriveSubsystem;

/**
 * 
 */
public class ShootCommand extends Command {
    // Subsystems
    private final SwerveDriveSubsystem swerveDriveSubsystem;
    private final ShooterSubsystem shooterSubsystem;
    private final HopperSubsystem hopperSubsystem;
    private final FeederSubsystem feederSubsystem;

    /**
     * 
     */
    public ShootCommand(SwerveDriveSubsystem swerveDriveSubsystem, ShooterSubsystem shooterSubsystem,
            HopperSubsystem hopperSubsystem, FeederSubsystem feederSubsystem) {
        this.swerveDriveSubsystem = swerveDriveSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.hopperSubsystem = hopperSubsystem;
        this.feederSubsystem = feederSubsystem;

        addRequirements(shooterSubsystem, hopperSubsystem, feederSubsystem);
    }

    @Override
    public void initialize() {
        // Only allow slow driving
        swerveDriveSubsystem.setSlowMode(true);
    }

    @Override
    public void execute() {
        // LockTurretOnTarget and ShooterSubsystem handle aiming and setting shooter RPM

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
