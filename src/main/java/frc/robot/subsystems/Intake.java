package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.Intake.IntakeExtendArm;
import frc.robot.commands.Intake.IntakeRetractArm;
import frc.robot.commands.Intake.IntakeSpinMotor;
import frc.robot.commands.Intake.OuttakeSpinMotor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class Intake extends SubsystemBase {
    SparkMax intakeMotor;
    SparkMax armMotor;
    int intakeMotorID = 60;
    int armMotorID = 5;
    
private static DigitalInput retractingLimitSwitch = new DigitalInput(7);
private static DigitalInput extendingLimitSwitch = new DigitalInput(8);

 static ShuffleboardTab tab = Shuffleboard.getTab("intakeMotor");

     
    private static Intake mInstance;
     public static Intake getInstance() {
        if (mInstance == null) {
            mInstance = new Intake();
        }
        return mInstance;
     }
    

    private Intake() {
        intakeMotor = new SparkMax(intakeMotorID, SparkMax.MotorType.kBrushless);
        armMotor = new SparkMax(armMotorID, SparkMax.MotorType.kBrushless);

        tab.addBoolean("Extending",() -> extendingLimitSwitch.get());
        tab.addBoolean("Retracting",() -> retractingLimitSwitch.get());
    }



        
    public void setIntakeSpeed(double speed) {
        intakeMotor.set(speed);
    }

    public void setArmSpeed(double speed) {
        armMotor.set(speed);
    }

    // public Command IntakeSpin() {
    //     return new IntakeSpinMotor(this);
    // }

    // public Command OuttakeSpin() {
    //     return new OuttakeSpinMotor(this);
    // }

    // public Command ExtendSpin() {
    //     return new IntakeExtendArm(this);
    // }

    // public Command RetractSpin() {
    //     return new IntakeRetractArm(this);
    // }

     public boolean isIntakeRetracted() {
        return !retractingLimitSwitch.get();
    }

    public boolean isIntakeExtended() {
        return !extendingLimitSwitch.get();
    }
}


