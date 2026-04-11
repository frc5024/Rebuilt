package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
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
    // Parameters
    private final SwerveDriveSubsystem swerveDriveSubsystem;
    private final HopperSubsystem hopperSubsystem;
    private final FeederSubsystem feederSubsystem;
    private final ShooterSubsystem shooterSubsystem;
    private final boolean isIntaking;
    private final double timeLimitSeconds;

    // Variables
    private Timer runTimer;

    /**
     * 
     */
    public ShootCommand(SwerveDriveSubsystem swerveDriveSubsystem, HopperSubsystem hopperSubsystem,
            FeederSubsystem feederSubsystem, ShooterSubsystem shooterSubsystem, boolean isIntaking,
            double timeLimitSeconds) {
        this.swerveDriveSubsystem = swerveDriveSubsystem;
        this.hopperSubsystem = hopperSubsystem;
        this.feederSubsystem = feederSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.isIntaking = isIntaking;
        this.timeLimitSeconds = timeLimitSeconds;

        this.runTimer = new Timer();

        addRequirements(hopperSubsystem, feederSubsystem, shooterSubsystem);
    }

    @Override
    public void initialize() {
        // Only allow slow driving
        swerveDriveSubsystem.setSlowMode(true);

        // reset and start timer
        runTimer.reset();
        runTimer.start();
    }

    @Override
    public void execute() {
        // LockTurretOnTarget and ShooterSubsystem handle aiming and setting shooter RPM

        // spin up hooper and feeder if shooter is ready
        if (shooterSubsystem.isAtSetpoint()) {
            feederSubsystem.setVelocity(FeederConstants.RPM);
            hopperSubsystem.setVelocity(isIntaking ? HopperConstants.SLOW_RPM : HopperConstants.RPM);
        }
    }

    @Override
    public void end(boolean interrupted) {
        runTimer.stop();
        feederSubsystem.setVelocity(0.0);
        hopperSubsystem.setVelocity(0.0);
        shooterSubsystem.setVelocity(ShooterConstants.IDLE_SPEED_RPM);
        swerveDriveSubsystem.setSlowMode(false);
    }

    @Override
    public boolean isFinished() {
        // run until the time limit reached or the trigger is released
        return timeLimitSeconds > 0.0 ? runTimer.isRunning() && runTimer.hasElapsed(timeLimitSeconds) : false;
    }
}
