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
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.Turret.StickRotationCommand;
import frc.robot.commands.Turret.spinCommand;
import frc.robot.commands.Turret.negativeSpin;
import frc.robot.commands.Turret.spinToAngleCommand;
import frc.robot.subsystems.swervedrive.SwerveDriveSubsystem;
import frc.robot.commands.Turret.resetSetpoint;
import frc.robot.Constants;
import frc.robot.Constants.turretConstants;
// import frc.robot.commands.Turret.LockontoTargetCommand;
import frc.robot.commands.Turret.LockSetpointCommand;
import frc.robot.subsystems.swervedrive.SwerveDriveSubsystem;

public class Turret extends SubsystemBase {

    private final SparkMax turretMotor;
    private final ProfiledPIDController pidController;
    // private final AHRS gyro;
    private TrapezoidProfile.Constraints feedForwardConstraints;
    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(turretConstants.kS,
            turretConstants.kV, turretConstants.kA); // Tune these values

    private double outputSpeed;
    private double joystickSpeed;
    private double currentAngle;
    private double targetAngle;

    private double voltageValue;

    private DigitalInput hallEffect;

    private static final double GEAR_RATIO = 10.75;

    public final int rotationAxis = XboxController.Axis.kRightX.value;

    // int hallEffectChannel = Constants.turretConstants.hallEffectChannel;

    ShuffleboardTab tab = Shuffleboard.getTab("Turret");
    GenericEntry pEntry = tab.add("SET P", turretConstants.kP).getEntry();
    GenericEntry dEntry = tab.add("SET D", turretConstants.kD).getEntry();
    GenericEntry iEntry = tab.add("SET I", turretConstants.kI).getEntry();

    GenericEntry sEntry = tab.add("SET S", turretConstants.kS).getEntry();
    GenericEntry vEntry = tab.add("SET V", turretConstants.kV).getEntry();
    GenericEntry aEntry = tab.add("SET A", turretConstants.kA).getEntry();

    GenericEntry targetAngleEntry = tab.add("SET target angle", turretConstants.targetAngle).getEntry();

    GenericEntry maxSpeedEntry = tab.add("SET max speed", turretConstants.turretMaxSpeed).getEntry();
    GenericEntry maxAccelEntry = tab.add("SET max accel", turretConstants.turretMaxAccel).getEntry();
    GenericEntry toleranceEntry = tab.add("SET TOLERANCE", turretConstants.turretTolerance).getEntry();

    private static Turret mInstance;

    public static Turret getInstance() {
        if (mInstance == null) {
            mInstance = new Turret();
        }
        return mInstance;
    }

    public Turret() {

        turretMotor = new SparkMax(Constants.turretConstants.turretMotorChannel, SparkLowLevel.MotorType.kBrushless);
        // setDefaultCommand(new StickRotationCommand(this));

        // hallEffect = new DigitalInput(hallEffectChannel);

        feedForwardConstraints = new TrapezoidProfile.Constraints(turretConstants.turretMaxSpeed,
                turretConstants.turretMaxAccel);

        pidController = new ProfiledPIDController(Constants.turretConstants.kP,
                Constants.turretConstants.kI, Constants.turretConstants.kD, feedForwardConstraints);
        // Enable continuous input so the controller will take the shortest path across
        // the -180/180 wrap
        pidController.enableContinuousInput(-180.0, 180.0);
        pidController.setTolerance(Constants.turretConstants.turretTolerance);
        // gyro = new AHRS(SPI.Port.kMXP);

        tab.addDouble("current angle", () -> getTurretAngle());

        tab.addDouble("current velocity", () -> getCurrentVelocity());

        tab.addDouble("Estimated Velocity", () -> pidController.getSetpoint().velocity);

        tab.addDouble("Setpoint", () -> pidController.getSetpoint().position);
        
        tab.addDouble("voltageValue", () -> voltageValue);

        pEntry.setDouble(Constants.turretConstants.kP);
        iEntry.setDouble(Constants.turretConstants.kI);
        dEntry.setDouble(Constants.turretConstants.kD);
        vEntry.setDouble(Constants.turretConstants.kV);
        toleranceEntry.setDouble(Constants.turretConstants.turretTolerance);

    }

    public boolean pidEnabled;

    public void enablePID() {
        pidEnabled = true;
        System.out.println("PID enabled for turret");
    }

    public void disablePID() {
        pidEnabled = false;
        turretMotor.set(0);
        System.out.println("PID disabled for turret");
    }

    public void updateJoystick(double joystickSpeed) {
        this.joystickSpeed = joystickSpeed;
    }

    public void runTurret(double speed) {
        turretMotor.set(speed);
    }

    public void updatePID() {
        // double currentAngle = getTurretAngle();
        // double pidOutput = pidController.calculate(currentAngle, targetAngle);

        // // Calculate feedforward (velocity is approximately 0 for position control)
        // double feedforwardOutput = feedforward.calculate(0.1);

        // double totalOutput = Math.max(-1, Math.min(1, pidOutput));
        // double turretOutput = totalOutput + feedforwardOutput;

        System.out.println("updating pid");

        double pidValue = pidController.calculate(getTurretAngle());
        double feedforwardValue = feedforward.calculate(pidController.getSetpoint().velocity);

        voltageValue = pidValue
                + feedforwardValue;

        if (Math.abs(voltageValue) > turretConstants.turretMaxSpeed) {
            turretMotor.setVoltage(turretConstants.turretMaxSpeed * Math.signum(voltageValue));
        } else {
            turretMotor.setVoltage(voltageValue);
        }

        // System.out.println("pidvalue: " + pidValue);
        // System.out.println("ff value: " + feedforwardValue);
        // System.out.println("voltage value: " + voltageValue);
    }

    @Override
    public void periodic() {

        pidController.setP(pEntry.getDouble(turretConstants.kP));
        pidController.setI(iEntry.getDouble(turretConstants.kI));
        pidController.setD(dEntry.getDouble(turretConstants.kD));

        feedForwardConstraints = new TrapezoidProfile.Constraints(
                maxSpeedEntry.getDouble(turretConstants.turretMaxSpeed),
                maxAccelEntry.getDouble(turretConstants.turretMaxAccel));

        pidController.setConstraints(feedForwardConstraints);

        feedforward.setKs(sEntry.getDouble(turretConstants.kS));
        feedforward.setKv(vEntry.getDouble(turretConstants.kV));
        feedforward.setKa(aEntry.getDouble(turretConstants.kA));

        // SimpleMotorFeedforward.setV(vEntry.getDouble(turretConstants.kV));

        // System.out.println("hello, pid is running");
        if (pidEnabled) {
            // System.out.println("hello, pid is UPDATEDDDDDDDDDDDDDDDDDDDDDDDDDDDD");
            updatePID();
        }
    }

    // public void turretMath(double joystickSpeed) {
    // if(Math.abs(joystickSpeed) > 0.01) {
    // outputSpeed = joystickSpeed;
    // } else {
    // outputSpeed = 0;
    // }
    // turretMotor.set(outputSpeed);
    // }

    public double getTurretAngle() {
        double motorRotations = turretMotor.getEncoder().getPosition();
        double turretRotations = motorRotations / GEAR_RATIO;
        // returns degrees normalized to [-180, 180)
        double angle = turretRotations * 360.0;
        while (angle >= 180.0) {
            angle -= 360.0;
        }
        while (angle < -180.0) {
            angle += 360.0;
        }
        return angle;
    }

    /**
     * 
     * @return
     */
    public double getCurrentVelocity() {
        // double currentVelocity = turretMotor.getEncoder().getVelocity();
        // double DegPerSec = currentVelocity * 360/60;
        // return DegPerSec;

        // turretMotor.getEncoder().getVelocity() returns motor RPM (rotations per
        // minute)
        double motorRPM = turretMotor.getEncoder().getVelocity();
        // Convert motor RPM -> turret rotations per second by dividing by gear ratio
        // and 60
        double turretRPS = (motorRPM / GEAR_RATIO) / 60.0;
        // Convert rotations per second -> degrees per second
        double degPerSec = turretRPS * 360.0;
        return degPerSec;
    }

    public double getTargetVelocity() {
        double targetVelocity = pidController.getSetpoint().velocity;
        return targetVelocity;
    }

    public void zeroEncoder() {
        turretMotor.getEncoder().setPosition(0.0);

    }

    public void setTargetAngle(double targetAngle) {
        pidController.setGoal(targetAngle);
    }

    public void resetPID() {
        // reset the controller to the current measured angle
        // pidController.reset(currentAngle);
        pidController.reset(getTurretAngle());
    }

    public boolean isAtTargetAngle() {
        if (pidController.atGoal() == true) {
            System.out.println("@ goal");
        }
        return pidController.atGoal();
    }

    // public void setTargetAngle(double targetAngle) {
    // this.targetAngle = targetAngle; // Update the desired turret angle
    // }

    // public void updateTurretAngle() {
    // currentAngle = getTurretAngle();

    // double output = pidController.calculate(currentAngle, targetAngle);

    // turretMotor.set(output);
    // }

    // public double getRobotHeading() {
    // return gyro.getAngle();
    // }

    public void setIdle() {
        turretMotor.set(0);
        joystickSpeed = 0;
    }

    // public Command stickRotation() {
    // return new StickRotationCommand(this);
    // }

    // public Command lockontoTargetCommand() {
    // return new LockontoTargetCommand(this);
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
