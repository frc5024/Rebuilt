// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.feederJamCommand;
import frc.robot.commands.feederCommand;

public class Feeder extends SubsystemBase {

  private SparkMax feederMotor;

  public static Feeder mInstance = null;

  public static Feeder getInstance() {
    if (mInstance == null) {
      mInstance = new Feeder();
    }
    return mInstance;
  }

  public Feeder() {
    feederMotor = new SparkMax(3, MotorType.kBrushless);
  }

  public void setIdle() {
    feederMotor.set(0);
  }

  public void setFeederSpeed(double feederspeed) {
    feederMotor.set(-feederspeed);
  }

  public Command feederCommand() {
    return new feederCommand(this);
  }

  public Command jammedCommand() {
    return new feederJamCommand(this);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
