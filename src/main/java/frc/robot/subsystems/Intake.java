package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.IntakeSpinMotor;
import frc.robot.commands.OuttakeSpinMotor;
import edu.wpi.first.wpilibj2.command.Command;

public class Intake extends SubsystemBase {
    SparkMax intakeMotor;
    int motorID = 5;
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
        
    public void setSpeed(double Speed) {
        intakeMotor.set(Speed);

    }

    public Command IntakeSpin() {
        return new IntakeSpinMotor(this);
    }

    public Command OuttakeSpin() {
        return new OuttakeSpinMotor(this);
    }
}


