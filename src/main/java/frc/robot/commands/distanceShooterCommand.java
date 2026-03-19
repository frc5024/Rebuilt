// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.SwerveDriveSubsystem;
import frc.robot.util.GameUtil;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class distanceShooterCommand extends Command {

    public final ShooterSubsystem shootersubsystem;
    public final SwerveDriveSubsystem swerveDriveSubsystem;

    // public Supplier<Double> setVelocity;

    public distanceShooterCommand(ShooterSubsystem shootersubsystem, SwerveDriveSubsystem swerveDriveSubsystem) {
        this.shootersubsystem = shootersubsystem;
        this.swerveDriveSubsystem = swerveDriveSubsystem;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        shootersubsystem.setPidEnabled(true);

    }

    @Override
    public void execute() {
        Pose2d hubPose = GameUtil.getHubPose();

        double distance = (swerveDriveSubsystem.getPose().getTranslation().getDistance(hubPose.getTranslation()));
        double RPM = Constants.ShooterConstants.velocityToRPMMap.get(distance);
        // TESTING RPM IN LIBRARY
        // RPM *= 0.5;
        SmartDashboard.putNumber("Distance", distance);
        SmartDashboard.putNumber("RPM", RPM);
        Logger.recordOutput("shooting/distance", distance);
        Logger.recordOutput("shooting/RPM", RPM);

        shootersubsystem.setShooterPID(RPM);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        shootersubsystem.setPidEnabled(false);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

}
