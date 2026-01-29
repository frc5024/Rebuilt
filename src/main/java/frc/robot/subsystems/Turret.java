package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
//import com.kauailabs.navx.frc.AHRS;
//import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.Turret.StickRotationCommand;
import frc.robot.commands.Turret.spinCommand;
import frc.robot.commands.Turret.negativeSpin;
import frc.robot.commands.Turret.spinToAngleCommand;
import frc.robot.commands.Turret.resetSetpoint;
import frc.robot.Constants;
import frc.robot.Constants.turretConstants;
//import frc.robot.commands.Turret.LockontoTargetCommand;

public class Turret extends SubsystemBase {


    private final SparkMax turretMotor;
    private final ProfiledPIDController pidController;
    //private final AHRS gyro;
    private TrapezoidProfile.Constraints feedForwardConstraints;
    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(turretConstants.kS, turretConstants.kV, turretConstants.kA); // Tune these values

    private double outputSpeed;
    private double joystickSpeed;
    private double currentAngle;
    private double targetAngle;

    private double voltageValue;

 

    private static final double GEAR_RATIO = 7.5;

    public final int rotationAxis = XboxController.Axis.kRightX.value;

    ShuffleboardTab tab = Shuffleboard.getTab("Turret");
    GenericEntry pEntry = tab.add("SET P", turretConstants.kP).getEntry();
    GenericEntry dEntry = tab.add("SET D", turretConstants.kD).getEntry();
    GenericEntry iEntry = tab.add("SET I", turretConstants.kI).getEntry();

    GenericEntry kEntry = tab.add("SET S", turretConstants.kS).getEntry();
    GenericEntry vEntry = tab.add("SET V", turretConstants.kV).getEntry();
    GenericEntry aEntry = tab.add("SET A", turretConstants.kA).getEntry();

    GenericEntry maxSpeedEntry = tab.add("SET max speed", turretConstants.turretMaxSpeed).getEntry();
    GenericEntry maxAccelEntry = tab.add("SET max accel", turretConstants.turretMaxAccel).getEntry();
    GenericEntry toleranceEntry = tab.add("SET TOLERANCE", turretConstants.turretTolerance).getEntry();


    public Turret() {
        

        turretMotor = new SparkMax(Constants.turretConstants.turretMotorChannel, SparkLowLevel.MotorType.kBrushless);
        setDefaultCommand(new StickRotationCommand(this));

        feedForwardConstraints = new TrapezoidProfile.Constraints(turretConstants.turretMaxSpeed,
                turretConstants.turretMaxAccel);

        pidController = new ProfiledPIDController(Constants.turretConstants.kP, 
        Constants.turretConstants.kI, Constants.turretConstants.kD, feedForwardConstraints);
        //pidController.enableContinuousInput(-180.0, 180.0);
        pidController.setTolerance(Constants.turretConstants.turretTolerance);
        //gyro = new AHRS(SPI.Port.kMXP);

        tab.addDouble("current angle", () -> getTurretAngle());

        
        

        pEntry.setDouble(Constants.turretConstants.kP);
        iEntry.setDouble(Constants.turretConstants.kI);
        dEntry.setDouble(Constants.turretConstants.kD);
        vEntry.setDouble(Constants.turretConstants.kV);
        toleranceEntry.setDouble(Constants.turretConstants.turretTolerance);

        
    }
 
    public boolean pidEnabled;

    public void enablePID() {
        pidEnabled = true;
    }

    public void disablePID() {
        pidEnabled = false;
        turretMotor.set(0);
    }

    public void updateJoystick(double joystickSpeed) {
        this.joystickSpeed = joystickSpeed;
    }

    public void runTurret(double speed) {
        turretMotor.set(speed);
    }

    public void updatePID() {
        //double currentAngle = getTurretAngle();
        // double pidOutput = pidController.calculate(currentAngle, targetAngle);
        
        // // Calculate feedforward (velocity is approximately 0 for position control)
        // double feedforwardOutput = feedforward.calculate(0.1);
        
        // double totalOutput = Math.max(-1, Math.min(1, pidOutput));
        // double turretOutput = totalOutput + feedforwardOutput;

        voltageValue = pidController.calculate(getTurretAngle())
                + feedforward.calculate(pidController.getSetpoint().velocity);
        //turretMotor.setVoltage(voltageValue);
        //turretMotor.set(turretOutput);
    }

    @Override
    public void periodic() {

        pidController.setP(pEntry.getDouble(turretConstants.kP));
        pidController.setI(iEntry.getDouble(turretConstants.kI));
        pidController.setD(dEntry.getDouble(turretConstants.kD));   
        //SimpleMotorFeedforward.setV(vEntry.getDouble(turretConstants.kV));

      //System.out.println("hello, pid is running");
        if (pidEnabled) {
            System.out.println("hello, pid is UPDATEDDDDDDDDDDDDDDDDDDDDDDDDDDDD");
            updatePID();
        }
    }


    // public void turretMath(double joystickSpeed) {
    //     if(Math.abs(joystickSpeed) > 0.01) {
    //         outputSpeed = joystickSpeed;
    //     } else {
    //         outputSpeed = 0;
    //     }
    //     turretMotor.set(outputSpeed);
    // }

    public double getTurretAngle() {
        double motorRotations = turretMotor.getEncoder().getPosition();
        double turretRotations = motorRotations / GEAR_RATIO;
        return turretRotations * 360.0; //returns degrees
    }
    /**
     * 
     * @return
     */
    public double getCurrentVelocity() {
        double currentVelocity = turretMotor.getEncoder().getVelocity();
        double DegPerSec = currentVelocity * 360/60; 
        return DegPerSec; 
    }

    public void zeroEncoder() {
        turretMotor.getEncoder().setPosition(0.0);
        
    }

    public void setTargetAngle(double targetAngle) {
        pidController.setGoal(targetAngle);
    }

    public void zeroSetpoint() {
        pidController.reset(currentAngle);
    }

    public boolean isAtTargetAngle() {
        return pidController.atGoal();
    }

    // public void setTargetAngle(double targetAngle) {
    //     this.targetAngle = targetAngle; // Update the desired turret angle
    // }



    // public void updateTurretAngle() {
    //     currentAngle = getTurretAngle();

    //     double output = pidController.calculate(currentAngle, targetAngle);

    //     turretMotor.set(output);
    // }


    // public double getRobotHeading() {
    //     return gyro.getAngle(); 
    // }

    public void setIdle() {
        turretMotor.set(0);
        joystickSpeed = 0;
    }


    public Command stickRotation(){
        return new StickRotationCommand(this);
    }

    // public Command lockontoTargetCommand(){
    //     return new LockontoTargetCommand(this);
    // }

    public Command spinCommand() {
        return new spinCommand(this);
    }

    public Command negativeSpin() {
        return new negativeSpin(this);
    }

    public Command spinToAngleCommand(double angle) {
        return new spinToAngleCommand(this, angle);
    }

    public Command resetSetpoint() {
        return new resetSetpoint(this);
    }

    public void setDefaultCommand(Turret turretSubsystem, Object object) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setDefaultCommand'");
    }

}

