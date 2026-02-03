package frc.robot.subsystems;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.swerve.utility.WheelForceCalculator.Feedforwards;
import com.google.flatbuffers.Constants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkFlex;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.shooterConstants;
import frc.robot.commands.shooterCommand;

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

    private boolean enabled;
    private double setVelocity;

   private final SparkBaseConfig flywheel1MotorConfig = new SparkFlexConfig()
            .idleMode(IdleMode.kCoast); // sets the motors to coast mode
            //.inverted(true);
    private final SparkBaseConfig flywheel2MotorConfig = new SparkFlexConfig()
            .idleMode(IdleMode.kCoast)
            // .inverted(true)
            .follow(51, false);
    
    private PIDController PID;
    private SimpleMotorFeedforward feedForward; 

    ShuffleboardTab tab = Shuffleboard.getTab("Shooter");
    GenericEntry pEntry = tab.add("SET P", shooterConstants.kP).getEntry();
    GenericEntry dEntry = tab.add("SET D", shooterConstants.kD).getEntry();
    GenericEntry iEntry = tab.add("SET I", shooterConstants.kI).getEntry();
    GenericEntry gEntry = tab.add("SET G", shooterConstants.kS).getEntry();
    GenericEntry vEntry = tab.add("SET V", shooterConstants.kV).getEntry();
    GenericEntry aEntry = tab.add("SET A", shooterConstants.kA).getEntry();

    public shooter(){
        flywheel1 = new SparkFlex(51, MotorType.kBrushless);
        flywheel2 = new SparkFlex(52, MotorType.kBrushless);

        RelativeEncoder encoder = flywheel1.getEncoder();

        

        PID = new PIDController(shooterConstants.kP, shooterConstants.kI, shooterConstants.kD);

        feedForward = new SimpleMotorFeedforward(shooterConstants.kS, shooterConstants.kV, shooterConstants.kA);
    }

    @Override
    public void periodic() {
         if (enabled) {
            setPIDMotor();
        } else {
            flywheel1.set(0);
        }

    }

    

    public void setPIDMotor() {
        setVelocity = PID.calculate(flywheel1.getEncoder().getVelocity()) + feedForward.calculate(PID.getSetpoint());
        flywheel1.set(setVelocity);
    }


    public Command shooterCommand() {

        return new shooterCommand(this, shooterConstants.setVelocity);
    }

   
}
