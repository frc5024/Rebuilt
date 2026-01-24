// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.shooterConstants;
import frc.robot.subsystems.feeder;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class feederCommand extends Command {
  private final feeder feederSubsystem;

   ShuffleboardTab tab = Shuffleboard.getTab("feederMotor");
    GenericEntry pEntry = tab.add("SET FEEDSPEED", shooterConstants.feederspeed).getEntry();

  public feederCommand(feeder feederSubsystem) {
    this.feederSubsystem = feederSubsystem;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    feederSubsystem.setIdle();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    feederSubsystem.setFeederSpeed(pEntry.getDouble(0.1));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    feederSubsystem.setIdle();
  }

}
