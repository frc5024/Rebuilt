package frc.robot.commands;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.util.GameUtil;

/**
 * 
 */
public class ShootCommand extends Command {
    // Subsystems
    private final ShooterSubsystem shooterSubsystem;
    private final HopperSubsystem hopperSubsystem;
    private final FeederSubsystem feederSubsystem;
    private final Supplier<Pose2d> robotPoseSupplier;

    // Variables
    private Pose2d targetPose;

    /**
     * 
     */
    public ShootCommand(ShooterSubsystem shooterSubsystem, HopperSubsystem hopperSubsystem,
            FeederSubsystem feederSubsystem, Supplier<Pose2d> robotPoseSupplier) {
        this.shooterSubsystem = shooterSubsystem;
        this.hopperSubsystem = hopperSubsystem;
        this.feederSubsystem = feederSubsystem;
        this.robotPoseSupplier = robotPoseSupplier;

        addRequirements(shooterSubsystem, hopperSubsystem, feederSubsystem);
    }

    @Override
    public void initialize() {
        // get the target we want to shoot at
        Pose2d robotPose = robotPoseSupplier.get();
        targetPose = GameUtil.getTargetPose(robotPose);

        // get distance to target and cooresponding RPM
        double distance = robotPose.getTranslation().getDistance(targetPose.getTranslation());
        double rpm = Constants.ShooterConstants.velocityToRPMMap.get(distance);

        // set the shooter RPM
        shooterSubsystem.setVelocity(rpm);

        Logger.recordOutput("Shooter/DistanceToTarget", distance);
    }

    @Override
    public void execute() {
        // spin up hooper and feeder if shooter is ready
        if (shooterSubsystem.isAtSetpoint()) {
            feederSubsystem.start();
            hopperSubsystem.start();
        }

        Logger.recordOutput("Shooter/Active Command", this.getName());
    }

    @Override
    public void end(boolean interrupted) {
        feederSubsystem.stop();
        hopperSubsystem.stop();
        shooterSubsystem.setVelocity(ShooterConstants.IDLE_SPEED_RPM);

        Logger.recordOutput("Shooter/Active Command", "");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
