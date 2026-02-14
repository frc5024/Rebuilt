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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.shooterConstants;
import frc.robot.commands.shooterCommand;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;


public class shooter extends SubsystemBase{
    private SparkFlex flywheel1;
    private SparkFlex flywheel2;

    private boolean enabled;

     private static shooter mInstance = null;

    public static shooter getInstance() {
        if (mInstance == null) {
            mInstance = new shooter();
        }
        return mInstance;
    }

   private final SparkBaseConfig flywheel1MotorConfig = new SparkFlexConfig()
            .idleMode(IdleMode.kCoast) // sets the motors to coast mode
            .inverted(true);
    private final SparkBaseConfig flywheel2MotorConfig = new SparkFlexConfig()
            .idleMode(IdleMode.kCoast)
            .follow(51, true);
    
    private PIDController PID;
    private SimpleMotorFeedforward feedForward; 

    ShuffleboardTab tab = Shuffleboard.getTab("Shooter");
    GenericEntry pEntry = tab.add("SET P", shooterConstants.kP).getEntry();
    GenericEntry dEntry = tab.add("SET D", shooterConstants.kD).getEntry();
    GenericEntry iEntry = tab.add("SET I", shooterConstants.kI).getEntry();
    GenericEntry sEntry = tab.add("SET S", shooterConstants.kS).getEntry();
    GenericEntry vEntry = tab.add("SET V", shooterConstants.kV).getEntry();
    GenericEntry aEntry = tab.add("SET A", shooterConstants.kA).getEntry();
    GenericEntry setVelocityEntry = tab.add("SET VELOCITY", shooterConstants.setVelocity).getEntry();

    public shooter(){
        flywheel1 = new SparkFlex(51, MotorType.kBrushless);
        flywheel2 = new SparkFlex(52, MotorType.kBrushless);

        flywheel1.configure(flywheel1MotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        flywheel2.configure(flywheel2MotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        RelativeEncoder encoder = flywheel1.getEncoder();

        PID = new PIDController(shooterConstants.kP, shooterConstants.kI, shooterConstants.kD);

        feedForward = new SimpleMotorFeedforward(shooterConstants.kS, shooterConstants.kV, shooterConstants.kA);

        tab.addDouble("actual velocity", () -> flywheel1.getEncoder().getVelocity());
        tab.addDouble("actualVoltage", () -> flywheel1.getBusVoltage());
    }

    @Override
    public void periodic() { 
         if (enabled) {
            System.out.println("AAAAAAAAAAAAAAAAAAAAAAAAAAAAa");
            setPIDMotor();

        } else {
            flywheel1.set(0);
        }

    }

    public void setShooterPID(double setVelocity) {
        PID.setSetpoint(setVelocity);


    }

    public void setEnabled(boolean enabled) {
       this.enabled = enabled;
    }

    public void setPIDMotor() {

        double setVelocity = setVelocityEntry.getDouble(100);
        PID.setSetpoint(setVelocity);

        PID.setP(pEntry.getDouble(shooterConstants.kP));
        PID.setI(iEntry.getDouble(shooterConstants.kI));
        PID.setD(dEntry.getDouble(shooterConstants.kD));

        feedForward.setKs(sEntry.getDouble(shooterConstants.kS));
        feedForward.setKv(vEntry.getDouble(shooterConstants.kV));
        feedForward.setKa(aEntry.getDouble(shooterConstants.kA));


        double PIDoutput = PID.calculate(flywheel1.getEncoder().getVelocity());
        double feedForwardOutput = feedForward.calculate(PID.getSetpoint());
        double totalOutput = PIDoutput + feedForwardOutput;

        flywheel1.setVoltage(totalOutput);

         SmartDashboard.putNumber("PID", PIDoutput);
         SmartDashboard.putNumber("FeedForward", feedForwardOutput);
         SmartDashboard.putNumber("Total Output", totalOutput);
         

    }

    //public void getPosition() {
    //frc.robot.subsystems.swervedrive.SwerveDriveSubsystem.getPose();
    //}


    public Command shooterCommand() {

        return new frc.robot.commands.shooterCommand(this, shooterConstants.setVelocity);
    }

    //public Command runEverything() {
        //return new frc.robot.commands.runEverything(this, shooterConstants.setVelocity);
    //}

        
}
