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

public class HopperSubsystem extends SubsystemBase{
    public SparkMax hopperMotor;

    public static HopperSubsystem mInstance = null;
    public double speed;
    public boolean direction;

    ShuffleboardTab tab = Shuffleboard.getTab("Hopper");
    GenericEntry speedEntry = tab.add("SET SPEED", Constants.HopperConstants.speed).getEntry();

    public static HopperSubsystem getInstance() {
        if (mInstance == null) {
            mInstance = new HopperSubsystem();
        }
        return mInstance;
    }

    public HopperSubsystem() {
        hopperMotor = new SparkMax(Constants.HopperConstants.HopperMotorID, SparkLowLevel.MotorType.kBrushless);

        speedEntry.setDouble(Constants.HopperConstants.speed);
    }

    public void setSpeed(double speed) {
        hopperMotor.set(speed);
    }

    public void setIdle() {
        hopperMotor.set(0);
    }

    public Command SpinCommand() {
        return new Spin(this, 50);
    }

    public Command SpinEntryCommand() {
        return new Spin(this, speedEntry.getDouble(Constants.HopperConstants.speed));
    }
}
