// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.ClimbCommands.ClimbCommand;
import frc.robot.commands.ClimbCommands.DeclimbCommand;
import frc.robot.commands.ClimbCommands.PauseclimbCommand;

public class Climb extends SubsystemBase {
  private static Climb mInstance;

  private TalonFX climbMotor;




  public static final Climb getInstance() {
        if (mInstance == null) {
            mInstance = new Climb();

        }

        return mInstance;
    }

  /** Creates a new climb. */
  public Climb() {
     climbMotor = new TalonFX(3);
  }

  //Sets the speed of the climb motor to the inputted speel value
  public void setSpeed(Double speed) {
    climbMotor.set(speed);
    System.out.println(speed);
  }

  

  //Calls ClimbCommand to set climb motor speed to climb speed
  public Command climb(){
    return new ClimbCommand(this);
  }
  //Calls DeclimbCommand to set climb motor speed to declimb speed
  public Command declimb() {
    return new DeclimbCommand(this);
  }
  //Calls PauseclimbCommand to set climb motor speed to stopped
  public Command dontdeclimb() {
    return new PauseclimbCommand(this);
  }



  
 
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
