// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.HopperConstants;
import frc.robot.Constants.shooterConstants;
import frc.robot.commands.Hopper.Spin;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Hopper;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class runAll extends Command {
  private final Feeder feederSubsystem;
  private final Shooter shooterSubsystem;
  private final Hopper hopperSubsystem;

  static ShuffleboardTab tab1 = Shuffleboard.getTab("feederallMotor");
  static GenericEntry pEntry = tab1.add("SET allFEEDSPEED", shooterConstants.feederspeed).getEntry();
  static ShuffleboardTab tab2 = Shuffleboard.getTab("shooterallMotor");
  static GenericEntry dEntry = tab2.add("SET allSPEED", shooterConstants.speed).getEntry();
  static ShuffleboardTab tab3 = Shuffleboard.getTab("HopperallMotor");
  static GenericEntry iEntry = tab3.add("SET allHOPPERSPEED", HopperConstants.hopperSpeed).getEntry();

    public runAll(Feeder feederSubsystem, Shooter shooterSubsystem, Hopper hopperSubsystem) {
      this.feederSubsystem = feederSubsystem;
      this.shooterSubsystem = shooterSubsystem;
      this.hopperSubsystem = hopperSubsystem;
    
  }

  // Called when the command is initially scheduled.


  @Override
  public void initialize() {
    feederSubsystem.setIdle();
    shooterSubsystem.setIdle();
    hopperSubsystem.setIdle();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // shooterSubsystem.setMotorSpeed(dEntry.getDouble(0.1));
    // new WaitCommand(5);
    // feederSubsystem.setFeederSpeed(pEntry.getDouble(0.1));
    // hopperSubsystem.setSpeed(iEntry.getDouble(0.1));

    Commands.sequence(
      new shooterCommand(shooterSubsystem),  
      new WaitCommand(1),
      new feederCommand(feederSubsystem),
      new Spin(hopperSubsystem)
     );

     //SequentialCommandGroup ( 
      //new shooterCommand(shooterSubsystem),
      // new WaitCommand(1),
      // new feederCommand(feederSubsystem),
      // new Spin(hopperSubsystem)
      // );

  } 

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    feederSubsystem.setIdle();
    shooterSubsystem.setIdle();
    hopperSubsystem.setIdle();
  }

}