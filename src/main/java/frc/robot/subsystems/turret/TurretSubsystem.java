package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.turretConstants;
import frc.robot.commands.StickRotationCommand;

/**
 * 
 */
public class TurretSubsystem extends SubsystemBase {
    private final TurretModuleIO turretModuleIO;
    protected final TurretModuleIOInputsAutoLogged inputs;

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

    private static final double GEAR_RATIO = 10.75;

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

    /**
     * 
     */
    public TurretSubsystem(TurretModuleIO turretModuleIO) {
        this.turretModuleIO = turretModuleIO;
        this.inputs = new TurretModuleIOInputsAutoLogged();

        // setDefaultCommand(new StickRotationCommand(this,
        // RobotContainer.driverController));

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
        turretModuleIO.set(0);
        System.out.println("PID disabled for turret");
    }

    public void updateJoystick(double joystickSpeed) {
        this.joystickSpeed = joystickSpeed;
    }

    public void runTurret(double speed) {
        turretModuleIO.set(speed);
    }

    public void updatePID() {
        // double currentAngle = getTurretAngle();
        // double pidOutput = pidController.calculate(currentAngle, targetAngle);

        // // Calculate feedforward (velocity is approximately 0 for position control)
        // double feedforwardOutput = feedforward.calculate(0.1);

        // double totalOutput = Math.max(-1, Math.min(1, pidOutput));
        // double turretOutput = totalOutput + feedforwardOutput;

        System.out.println("updating pid");

        voltageValue = pidController.calculate(getTurretAngle())
                + feedforward.calculate(pidController.getSetpoint().velocity);
        turretModuleIO.setVoltage(voltageValue);
    }

    @Override
    public void periodic() {
        turretModuleIO.updateInputs(inputs);
        Logger.processInputs("Turret", inputs);

        pidController.setP(pEntry.getDouble(turretConstants.kP));
        pidController.setI(iEntry.getDouble(turretConstants.kI));
        pidController.setD(dEntry.getDouble(turretConstants.kD));
        // SimpleMotorFeedforward.setV(vEntry.getDouble(turretConstants.kV));

        // System.out.println("hello, pid is running");
        if (pidEnabled) {
            // System.out.println("hello, pid is UPDATEDDDDDDDDDDDDDDDDDDDDDDDDDDDD");
            updatePID();
        }
    }

    public double getCurrentDrawAmps() {
        return turretModuleIO.getCurrentDrawAmps();
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
        double motorRotations = turretModuleIO.getPosition();
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
        double motorRPM = turretModuleIO.getVelocity();
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
        turretModuleIO.setPosition(0.0);

    }

    public void setTargetAngle(double targetAngle) {
        pidController.setGoal(targetAngle);
    }

    public void zeroSetpoint() {
        // reset the controller to the current measured angle
        // pidController.reset(currentAngle);
        pidController.reset(getTurretAngle());
    }

    public boolean isAtTargetAngle() {
        System.out.println("@ goal");
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
        turretModuleIO.set(0);
    }

    public Command stickRotation(double speed) {
        return new StickRotationCommand(this, speed);
    }

}
