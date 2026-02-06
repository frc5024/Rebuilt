package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.commands.Hopper.Spin;
import frc.robot.Constants;

public class Hopper extends SubsystemBase{
    public SparkMax hopperMotor;

    public static Hopper mInstance = null;
    public double speed;
    public boolean direction;

    public static Hopper getInstance() {
        if (mInstance == null) {
            mInstance = new Hopper();
        }
        return mInstance;
    }

    public Hopper() {
        hopperMotor = new SparkMax(Constants.HopperConstants.HopperMotorID, SparkLowLevel.MotorType.kBrushless);

    }

    public void setSpeed(double speed) {
        hopperMotor.set(-speed);
    }

    public void setIdle() {
        hopperMotor.set(0);
    }

    public Command SpinCommand() {
        return new Spin(this);
    }

    // public Command SpinEntryCommand() {
    //     return new Spin(this, speedEntry.getDouble(Constants.HopperConstants.hopperSpeed));
    // }
}
