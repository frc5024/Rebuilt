package frc.robot.commands;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FeederConstants;
import frc.robot.Constants.HopperConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.util.GameUtil;

/**
 * 
 */
public class ShootForFiveSecondsCommand extends Command {
    // Subsystems
    private final ShooterSubsystem shooterSubsystem;
    private final HopperSubsystem hopperSubsystem;
    private final FeederSubsystem feederSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private final Supplier<Pose2d> robotPoseSupplier;

    // Variables
    private Pose2d targetPose;
    private Timer runTimer;

    /**
     * 
     */
    public ShootForFiveSecondsCommand(ShooterSubsystem shooterSubsystem, HopperSubsystem hopperSubsystem,
            FeederSubsystem feederSubsystem, IntakeSubsystem intakeSubsystem, Supplier<Pose2d> robotPoseSupplier) {
        this.shooterSubsystem = shooterSubsystem;
        this.hopperSubsystem = hopperSubsystem;
        this.feederSubsystem = feederSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.robotPoseSupplier = robotPoseSupplier;

        this.runTimer = new Timer();

        addRequirements(shooterSubsystem, hopperSubsystem, feederSubsystem);
    }

    @Override
    public void initialize() {
        // reset and start timer
        runTimer.reset();
        runTimer.start();

        // get the target we want to shoot at
        Pose2d robotPose = robotPoseSupplier.get();
        targetPose = GameUtil.getTargetPose(robotPose);

        // get distance to target and cooresponding RPM
        double distance = robotPose.getTranslation().getDistance(targetPose.getTranslation());
        double rpm = ShooterConstants.velocityToRPMMap.get(distance);

        // set the shooter RPM
        shooterSubsystem.setVelocity(rpm);

        Logger.recordOutput("Shooter/DistanceToTarget", distance);
    }

    @Override
    public void execute() {
        // spin up hooper and feeder if shooter is ready
        if (shooterSubsystem.isAtSetpoint()) {
            feederSubsystem.setVelocity(FeederConstants.RPM);
            hopperSubsystem.setVelocity(HopperConstants.RPM);
        }

        if (runTimer.hasElapsed(3)) {
            intakeSubsystem.retractArm();
        }

        Logger.recordOutput("Shooter/Active Command", this.getName());
    }

    @Override
    public void end(boolean interrupted) {
        runTimer.stop();
        feederSubsystem.setVelocity(0.0);
        hopperSubsystem.setVelocity(0.0);
        shooterSubsystem.setVelocity(ShooterConstants.IDLE_SPEED_RPM);

        Logger.recordOutput("Shooter/Active Command", "");
    }

    @Override
    public boolean isFinished() {
        // since we don't know when all the fuel has been launched we end the command
        // after 5 seconds
        return runTimer.isRunning() && runTimer.hasElapsed(5);
    }
}
