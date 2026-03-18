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
    private GenericEntry angleEntry;

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

            setAngle(angleEntry.getDouble(getCurrentAngle()));
        }

        if (pidEnabled) {
            turretModuleIO.setVoltage();
        }

        Logger.recordOutput("Turret/CurrentAngle", getCurrentAngle());
        Logger.recordOutput("Turret/SetPointAngle", turretModuleIO.getGoalPosition());
        Logger.recordOutput("Turret/AtTarget", isAtTarget());
        Logger.recordOutput("Turret/PIDEnabled", isPIDEnabled());
    }

    public boolean atGoal() {
        return turretModuleIO.atGoal();
    }

    public void disablePID() {
        pidEnabled = false;
        turretModuleIO.stop();
    }

    public void enablePID() {
        pidEnabled = true;
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
        disablePID();
        turretModuleIO.set(speed);
    }

    public void setAngle(double degrees) {
        turretModuleIO.setAngle(degrees);
        enablePID();
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

        double[] kPIDs = turretConstants.getPIDs();
        pEntry = tab.add("SET P", kPIDs[0]).getEntry();
        iEntry = tab.add("SET I", kPIDs[1]).getEntry();
        dEntry = tab.add("SET D", kPIDs[2]).getEntry();

        double[] kSVAs = turretConstants.getSVAs();
        sEntry = tab.add("SET S", kSVAs[0]).getEntry();
        vEntry = tab.add("SET V", kSVAs[1]).getEntry();
        aEntry = tab.add("SET A", kSVAs[2]).getEntry();

        maxSpeedEntry = tab.add("SET max speed", turretConstants.turretMaxSpeed).getEntry();
        maxAccelEntry = tab.add("SET max accel", turretConstants.turretMaxAccel).getEntry();
        toleranceEntry = tab.add("SET TOLERANCE", turretConstants.turretTolerance).getEntry();
        angleEntry = tab.add("SET ANGLE", 0.0).getEntry();

        pEntry.setDouble(kPIDs[0]);
        iEntry.setDouble(kPIDs[1]);
        dEntry.setDouble(kPIDs[2]);

        sEntry.setDouble(kSVAs[0]);
        vEntry.setDouble(kSVAs[1]);
        aEntry.setDouble(kSVAs[2]);

        maxSpeedEntry.setDouble(Constants.turretConstants.turretMaxSpeed);
        maxAccelEntry.setDouble(Constants.turretConstants.turretMaxAccel);
        toleranceEntry.setDouble(Constants.turretConstants.turretTolerance);
        angleEntry.setDouble(0.0);

        enablePID();
    }
}
