package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
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
import frc.robot.commands.spinToAngleCommand;

/**
 * 
 */
public class TurretSubsystem extends SubsystemBase {
    // private final TurretModuleIO turretModuleIO;
    // protected final TurretModuleIOInputsAutoLogged inputs;

    private final SparkMax turretMotor;

    private final ProfiledPIDController pidController;
    private TrapezoidProfile.Constraints feedForwardConstraints;
    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(turretConstants.kS,
            turretConstants.kV, turretConstants.kA); // Tune these values

    private double voltageValue;

    private static final double GEAR_RATIO = 28.6667 * 12;

    public final int rotationAxis = XboxController.Axis.kRightX.value;

    ShuffleboardTab tab = Shuffleboard.getTab("Turret");
    GenericEntry pEntry = tab.add("SET P", turretConstants.kP).getEntry();
    GenericEntry dEntry = tab.add("SET D", turretConstants.kD).getEntry();
    GenericEntry iEntry = tab.add("SET I", turretConstants.kI).getEntry();

    GenericEntry sEntry = tab.add("SET S", turretConstants.kS).getEntry();
    GenericEntry vEntry = tab.add("SET V", turretConstants.kV).getEntry();
    GenericEntry aEntry = tab.add("SET A", turretConstants.kA).getEntry();

    GenericEntry maxSpeedEntry = tab.add("SET max speed", turretConstants.turretMaxSpeed).getEntry();
    GenericEntry maxAccelEntry = tab.add("SET max accel", turretConstants.turretMaxAccel).getEntry();
    GenericEntry toleranceEntry = tab.add("SET TOLERANCE", turretConstants.turretTolerance).getEntry();

    double pValue;
    double fValue;

    /**
     * 
     */
    public TurretSubsystem(/* TurretModuleIO turretModuleIO */) {
        // this.turretModuleIO = turretModuleIO;
        // this.inputs = new TurretModuleIOInputsAutoLogged();
        turretMotor = new SparkMax(turretConstants.turretMotorChannel, SparkLowLevel.MotorType.kBrushless);

        feedForwardConstraints = new TrapezoidProfile.Constraints(turretConstants.turretMaxSpeed,
                turretConstants.turretMaxAccel);

        pidController = new ProfiledPIDController(Constants.turretConstants.kP,
                Constants.turretConstants.kI, Constants.turretConstants.kD, feedForwardConstraints);
        // Enable continuous input so the controller will take the shortest path across
        // the -180/180 wrap
        // pidController.enableContinuousInput(-135, 135);
        pidController.setTolerance(Constants.turretConstants.turretTolerance);
        // gyro = new AHRS(SPI.Port.kMXP);

        tab.addDouble("current angle", () -> getTurretAngle());

        tab.addDouble("goal", () -> pidController.getGoal().position);

        tab.addDouble("current velocity", () -> getCurrentVelocity());

        tab.addBoolean("pid enabled", () -> pidEnabled);

        tab.addDouble("voltage value", () -> voltageValue);

        tab.addDouble("pid value", () -> pValue);

        tab.addDouble("ff value", () -> fValue);

        tab.addBoolean("at target", () -> isAtTargetAngle());

        tab.addDouble("Estimated Velocity", () -> pidController.getSetpoint().velocity);

        pEntry.setDouble(Constants.turretConstants.kP);
        iEntry.setDouble(Constants.turretConstants.kI);
        dEntry.setDouble(Constants.turretConstants.kD);
        vEntry.setDouble(Constants.turretConstants.kV);
        maxSpeedEntry.setDouble(Constants.turretConstants.turretMaxSpeed);
        maxAccelEntry.setDouble(Constants.turretConstants.turretMaxAccel);
        toleranceEntry.setDouble(Constants.turretConstants.turretTolerance);

    }

    public boolean pidEnabled;

    public void enablePID() {
        pidEnabled = true;
        System.out.println("PID enabled for turret");
    }

    public void disablePID() {
        pidEnabled = false;
        // turretModuleIO.set(0);
        System.out.println("PID disabled for turret");
    }

    public void runTurret(double speed) {
        turretMotor.set(speed);
        // turretModuleIO.set(speed);
    }

    public void updatePID() {

        // double currentAngle = getTurretAngle();
        // double pidOutput = pidController.calculate(currentAngle, targetAngle);

        // // Calculate feedforward (velocity is approximately 0 for position control)
        // double feedforwardOutput = feedforward.calculate(0.1);

        // double totalOutput = Math.max(-1, Math.min(1, pidOutput));
        // double turretOutput = totalOutput + feedforwardOutput;

        // System.out.println("updating pid");

        pValue = pidController.calculate(getTurretAngle());

        fValue = feedforward.calculate(pidController.getSetpoint().velocity);

        voltageValue = pValue + fValue;

        // turretModuleIO.setVoltage(voltageValue);
        turretMotor.setVoltage(voltageValue);

    }

    @Override
    public void periodic() {
        // turretModuleIO.updateInputs(inputs);
        // Logger.processInputs("Turret", inputs);

        // turretModuleIO.setPID(pEntry.getDouble(turretConstants.kP),
        // iEntry.getDouble(turretConstants.kI),
        // dEntry.getDouble(turretConstants.kD));

        // turretModuleIO.setFF(sEntry.getDouble(turretConstants.kS),
        // vEntry.getDouble(turretConstants.kV),
        // aEntry.getDouble(turretConstants.kA));

        pidController.setP(pEntry.getDouble(turretConstants.kP));
        pidController.setI(iEntry.getDouble(turretConstants.kI));
        pidController.setD(dEntry.getDouble(turretConstants.kD));

        feedforward.setKv(vEntry.getDouble(turretConstants.kV));
        feedforward.setKs(sEntry.getDouble(turretConstants.kS));
        feedforward.setKa(aEntry.getDouble(turretConstants.kA));

        feedForwardConstraints = new TrapezoidProfile.Constraints(
                maxSpeedEntry.getDouble(turretConstants.turretMaxSpeed),
                maxAccelEntry.getDouble(turretConstants.turretMaxAccel));

        pidController.setConstraints(feedForwardConstraints);
        // feedforwardConstraints.setConstraints(maxSpeedEntry.getDouble(turretConstants.turretMaxSpeed));

        pidController.setTolerance(toleranceEntry.getDouble(turretConstants.turretTolerance));
        // SimpleMotorFeedforward.setV(vEntry.getDouble(turretConstants.kV));

        // System.out.println("hello, pid is running");
        if (pidEnabled) {
            // System.out.println("hello, pid is UPDATEDDDDDDDDDDDDDDDDDDDDDDDDDDDD");
            updatePID();
        }

        Logger.recordOutput("Turret/CurrentAngle", getTurretAngle());
        // Logger.recordOutput("Turret/SetPoint", turretModuleIO.getSetpoint());
    }

    // public double getCurrentDrawAmps() {
    // return turretModuleIO.getCurrentDrawAmps();
    // }

    public double getTurretAngle() {
        // double motorRotations = turretModuleIO.getPosition();
        double turretRotations = turretMotor.getEncoder().getPosition() / GEAR_RATIO;
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

        // double motorRPM = turretModuleIO.getVelocity();
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

    public void setAngle(double degrees) {
        // turretModuleIO.setAngle(degrees);

    }

    public void zeroEncoder() {
        // turretModuleIO.setPosition(0.0);
        turretMotor.getEncoder().setPosition(0.0);

    }

    public void setTargetAngle(double degrees) {

        pidController.setGoal(MathUtil.clamp(degrees, -90, 90));

    }

    public void zeroSetpoint() {
        // reset the controller to the current measured angle
        // pidController.reset(currentAngle);
        pidController.reset(getTurretAngle());
    }

    public boolean isAtTargetAngle() {
        // System.out.println("@ goal");
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
        // turretModuleIO.set(0);
        turretMotor.set(0);
    }

    public Command stickRotation(double speed) {
        return new StickRotationCommand(this, speed);
    }

    public Command spinToAngleCommand(double angle) {
        return new spinToAngleCommand(this, angle);
    }

}
