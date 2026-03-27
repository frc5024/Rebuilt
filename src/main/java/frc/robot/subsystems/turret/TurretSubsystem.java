package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.turretConstants;

/**
 * 
 */
public class TurretSubsystem extends SubsystemBase {
    // Advantagekit logging
    private final TurretModuleIO turretModuleIO;
    protected final TurretModuleIOInputsAutoLogged inputs;

    // Variables
    private boolean pidEnabled;

    // Shuffleboard entries
    private ShuffleboardTab tab;
    private GenericEntry pEntry;
    private GenericEntry dEntry;
    private GenericEntry iEntry;

    private GenericEntry sEntry;
    private GenericEntry vEntry;
    private GenericEntry aEntry;

    private GenericEntry maxSpeedEntry;
    private GenericEntry maxAccelEntry;
    private GenericEntry toleranceEntry;

    /**
     * 
     */
    public TurretSubsystem(TurretModuleIO turretModuleIO) {
        // set advantage kit IO logging
        this.turretModuleIO = turretModuleIO;
        this.inputs = new TurretModuleIOInputsAutoLogged();
        this.pidEnabled = false;

        // set shuffleboard entries if in tuning mode
        if (RobotConstants.TUNING_MODE) {
            setShuffleboard();
        }
    }

    @Override
    public void periodic() {
        // process hardware inputs
        turretModuleIO.updateInputs(inputs);
        Logger.processInputs("Turret", inputs);

        // update pid values if in tuning mode
        if (RobotConstants.TUNING_MODE) {
            turretModuleIO.setPID(
                    pEntry.getDouble(turretConstants.kP),
                    iEntry.getDouble(turretConstants.kI),
                    dEntry.getDouble(turretConstants.kD));

            turretModuleIO.setFF(
                    sEntry.getDouble(turretConstants.kS),
                    vEntry.getDouble(turretConstants.kV),
                    aEntry.getDouble(turretConstants.kA));

            turretModuleIO.setConstraints(maxSpeedEntry.getDouble(turretConstants.turretMaxSpeed),
                    maxAccelEntry.getDouble(turretConstants.turretMaxAccel),
                    toleranceEntry.getDouble(turretConstants.turretTolerance));
        }

        if (pidEnabled) {
            turretModuleIO.setVoltage();
        }

        Logger.recordOutput("Turret/CurrentAngle", getCurrentAngle());
        Logger.recordOutput("Turret/Velocity", turretModuleIO.getVelocity());
        Logger.recordOutput("Turret/GoalAngle", turretModuleIO.getGoalPosition());
        Logger.recordOutput("Turret/AtTarget", isAtTarget());
        Logger.recordOutput("Turret/PIDEnabled", isPIDEnabled());
    }

    public boolean atGoal() {
        return turretModuleIO.atGoal();
    }

    public void disablePID() {
        pidEnabled = false;
        turretModuleIO.set(0);
        System.out.println("PID disabled for turret");
    }

    public void enablePID() {
        pidEnabled = true;
        System.out.println("PID enabled for turret");
    }

    public double getCurrentDrawAmps() {
        return turretModuleIO.getCurrentDrawAmps();
    }

    public double getCurrentAngle() {
        return turretModuleIO.getCurrentAngle();
    }

    public boolean getHallEffectValue() {
        return turretModuleIO.getHallEffectValue();
    }

    public boolean isPIDEnabled() {
        return pidEnabled;
    }

    public void runTurret(double speed) {
        pidEnabled = false;
        turretModuleIO.set(speed);
    }

    public void setAngle(double degrees) {
        turretModuleIO.setAngle(degrees);
    }

    public void setPosition(double position) {
        turretModuleIO.setPosition(position);
    }

    public void zeroEncoder() {
        turretModuleIO.setPosition(0.0);
    }

    /**
     * Checks if the turret is locked on the hub (at target angle).
     * 
     * @return true if turret angle is within tolerance of target, false otherwise
     */
    public boolean isAtTarget() {
        double currentAngle = getCurrentAngle();
        double goalAngle = turretModuleIO.getGoalPosition();
        double tolerance = turretConstants.turretTolerance;

        return Math.abs(currentAngle - goalAngle) <= tolerance;
    }

    /**
     * 
     */
    private void setShuffleboard() {
        tab = Shuffleboard.getTab("Turret");
        pEntry = tab.add("SET P", turretConstants.kP).getEntry();
        iEntry = tab.add("SET I", turretConstants.kI).getEntry();
        dEntry = tab.add("SET D", turretConstants.kD).getEntry();

        sEntry = tab.add("SET S", turretConstants.kS).getEntry();
        vEntry = tab.add("SET V", turretConstants.kV).getEntry();
        aEntry = tab.add("SET A", turretConstants.kA).getEntry();

        maxSpeedEntry = tab.add("SET max speed", turretConstants.turretMaxSpeed).getEntry();
        maxAccelEntry = tab.add("SET max accel", turretConstants.turretMaxAccel).getEntry();
        toleranceEntry = tab.add("SET TOLERANCE", turretConstants.turretTolerance).getEntry();

        double[] kPIDs = turretConstants.getPIDs();
        pEntry.setDouble(kPIDs[0]);
        iEntry.setDouble(kPIDs[1]);
        dEntry.setDouble(kPIDs[2]);

        vEntry.setDouble(Constants.turretConstants.kV);

        maxSpeedEntry.setDouble(Constants.turretConstants.turretMaxSpeed);
        maxAccelEntry.setDouble(Constants.turretConstants.turretMaxAccel);
        toleranceEntry.setDouble(Constants.turretConstants.turretTolerance);

        // tab.addDouble("current angle", () -> getCurrentAngle());
        // tab.addDouble("goal", () -> pidController.getGoal().position);
        // tab.addDouble("current velocity", () -> getCurrentVelocity());
        // tab.addBoolean("pid enabled", () -> pidEnabled);
        // tab.addBoolean("hall effect", () -> getHallEffect());
        // tab.addDouble("voltage value", () -> voltageValue);
        // tab.addDouble("pid value", () -> pValue);
        // tab.addDouble("ff value", () -> fValue);
        // tab.addDouble("encoder value", () -> getEncoderValues());
        // tab.addBoolean("at target", () -> isAtTargetAngle());
        // tab.addDouble("Estimated Velocity", () ->
        // pidController.getSetpoint().velocity);
        // tab.addDouble("Estimated Position", () ->
        // pidController.getSetpoint().position);
    }
}
