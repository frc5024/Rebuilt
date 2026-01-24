package frc.robot.subsystems;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.swerve.utility.WheelForceCalculator.Feedforwards;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkFlex;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

public class shooter extends SubsystemBase{
    private SparkFlex flywheel1;
    private SparkFlex flywheel2;

   private final SparkBaseConfig flywheel1MotorConfig = new SparkFlexConfig()
            .idleMode(IdleMode.kCoast) // sets the motors to coast mode
            //.inverted(true);
    private final SparkBaseConfig flywheel2MotorConfig = new SparkFlexConfig()
            .idleMode(IdleMode.kCoast)
            // .inverted(true)
            .follow(51, false);
    
    private PIDController PID;
    private SimpleMotorFeedforward feedForward; 

    public shooter(){
        flywheel1 = new SparkFlex(51, MotorType.kBrushless);
        flywheel2 = new SparkFlex(52, MotorType.kBrushless);

        RelativeEncoder encoder = flywheel1.getEncoder();

        

        PID = new PIDController(0, 0, 0);

        feedForward = new SimpleMotorFeedforward(0, 0, 0);
    }

    @Override
    public void periodic() {
         if (enabled) {
            feedPIDMotor();
        } else {
            flywheel1.set(0);
        }

    }

   
}
