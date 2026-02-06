// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import com.revrobotics.*;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.shooterConstants;

public class Shooter extends SubsystemBase {

    private SparkFlex shooterMotor1;
    private SparkFlex shooterMotor2;

    public static Shooter mInstance = null;

    public static Shooter getInstance() {
        if (mInstance == null) {
            mInstance = new Shooter();
        }
        return mInstance;
    }

    public Shooter() {

        shooterMotor1 = new SparkFlex(51, MotorType.kBrushless);
        shooterMotor2 = new SparkFlex(52, MotorType.kBrushless);

    }

    public void setIdle() {
        shooterMotor1.set(0);
        shooterMotor2.set(0);
    }

    public void setMotorSpeed(double speed) {
        shooterMotor1.set(-speed);
        shooterMotor2.set(speed);
    }

    public Command shooterCommand() {
        return new frc.robot.commands.shooterCommand(this);
    }
}
