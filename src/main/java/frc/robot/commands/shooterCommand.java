// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.shooterConstants;
import frc.robot.subsystems.shooter;
import frc.robot.subsystems.swervedrive.SwerveDriveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class shooterCommand extends Command {

  public final shooter shootersubsystem;
  public final SwerveDriveSubsystem swerveDriveSubsystem;

  public double setVelocity;

  public shooterCommand (shooter shootersubsystem, SwerveDriveSubsystem swerveDriveSubsystem, double setVelocity) {  
    this.shootersubsystem = shootersubsystem;
    this.setVelocity = setVelocity;
    this.swerveDriveSubsystem = swerveDriveSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shootersubsystem.setEnabled(true);


  }

  @Override
  public void execute() {
    //get the position of the robot (with getpose), cords of the hub, calculate the dstance between the two
    Distance fieldLength = Inches.of(651.22);
    Translation2d blueHubPosition = (new Translation2d(Inches.of(182.11), Inches.of(317.69 / 2.0)));
    Translation2d redHubPosition = (new Translation2d(fieldLength.minus(Inches.of(182.11)), Inches.of(317.69 / 2.0)));

    double distance = (swerveDriveSubsystem.getPose().getTranslation().getDistance(blueHubPosition));
    System.out.println(distance);
    
    shootersubsystem.setShooterPID(setVelocity);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shootersubsystem.setEnabled(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
