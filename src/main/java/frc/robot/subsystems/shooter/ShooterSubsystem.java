package frc.robot.subsystems.shooter;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {

  private SparkMax m_motor;
  boolean toggle = true;

  public ShooterSubsystem() {
    //If using the sim, we don't want to define a real motor or things will explode
    if (!RobotBase.isSimulation()) { 
        m_motor = new SparkMax(ShooterConstants.sparkMaxMotorID, MotorType.kBrushless);
        setSpeed(0.0); 
      } else { 
        m_motor = null; 
      }
}

  public void setSpeed(double speed) {
    if (!RobotBase.isSimulation()) {
      m_motor.set(speed);
    } else {
      System.out.println("Set the speed to "+speed);
    }
  }

  public void toggleFeeder() {
    if (toggle) {
      setSpeed(0.2);
      toggle = false;
      System.out.println("IT WORKS!!!!");
    } else {
      setSpeed(0.0);
      toggle = true;
      System.out.println("IT WORKS????");
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
