package frc.robot.commands;

import java.util.function.Supplier;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.TeleopConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.swervedrive.SwerveDriveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

public class FollowTargetCommand extends Command {

    SwerveDriveSubsystem swerveDriveSubsystem;
    VisionSubsystem visionSubsystem;
    ProfiledPIDController pidControllerOmega;
    ProfiledPIDController pidControllerX;
    ProfiledPIDController pidControllerY;
    Supplier<Pose2d> poseProvider;
    double omegaSpeed;
    double xSpeed;
    double ySpeed;

    public FollowTargetCommand(SwerveDriveSubsystem swerveDriveSubsystem, VisionSubsystem visionSubsystem,
            Supplier<Pose2d> poseProvider) {

        this.swerveDriveSubsystem = swerveDriveSubsystem;
        this.visionSubsystem = visionSubsystem;
        this.poseProvider = poseProvider;

        // PID controller for rotating the robot towards target
        pidControllerOmega = new ProfiledPIDController(VisionConstants.kP, VisionConstants.kI, VisionConstants.kD,
                TeleopConstants.OMEGA_CONSTRAINTS);

    }

    public Rotation2d getGoal() {
        // Get the yaw of the target relative to the crosshair of the camera
        Rotation2d pose = this.visionSubsystem.getTargetX(0);
        return (pose);
    }

    @Override
    public void initialize() {
        Rotation2d goal = getGoal();
        pidControllerOmega.setGoal(goal.getRadians());
    }

    @Override
    public void execute() {
        Pose2d robotPose = this.poseProvider.get();
        
        // Calculate PID 
        omegaSpeed = this.pidControllerOmega.calculate(robotPose.getRotation().getRadians());

        // Set speed to zero when goal is reached
        if (this.pidControllerOmega.atGoal()) {
            omegaSpeed = 0;
        }
        
        // Turn motors using the PID values
        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(0.0, 0.0, omegaSpeed,
                robotPose.getRotation());
        this.swerveDriveSubsystem.runVelocity(chassisSpeeds);
    }

    @Override
    public void end(boolean interrupted) {
        // Stop using the swerve subsystem
        swerveDriveSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        // When goal is reached finish the command
        return pidControllerOmega.atGoal();
    }
}