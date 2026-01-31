package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.IntakeSpinMotor;
import frc.robot.commands.OuttakeSpinMotor;
import frc.robot.commands.IntakeExtendArm;
import frc.robot.commands.IntakeRetractArm;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.DigitalInput;

public class Intake extends SubsystemBase {
    SparkMax intakeMotor;
    int motorID = 4; // ID on prototype board, subject to change
    
private static DigitalInput retractingLimitSwitch = new DigitalInput(7);
private static DigitalInput extendingLimitSwitch = new DigitalInput(3);
     
    private static Intake mInstance;
     public static Intake getInstance() {
        if (mInstance == null) {
            mInstance = new Intake();
        }
        return mInstance;
     }
    
    

    private Intake() {
        intakeMotor = new SparkMax(motorID, SparkMax.MotorType.kBrushless);
    }
        
    public void setSpeed(double speed) {
        intakeMotor.set(speed);
    }

    public Command IntakeSpin() {
        return new IntakeSpinMotor(this);
    }

    public Command OuttakeSpin() {
        return new OuttakeSpinMotor(this);
    }

    public Command ExtendSpin() {
        return new IntakeExtendArm(this);
    }

    public Command RetractSpin() {
        return new IntakeRetractArm(this);
    }

     public boolean isIntakeRetracted() {
        return !retractingLimitSwitch.get();
    }

    public boolean isIntakeExtended() {
        return !extendingLimitSwitch.get();
    }
}


