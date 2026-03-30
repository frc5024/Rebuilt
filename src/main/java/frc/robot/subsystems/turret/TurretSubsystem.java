package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.TurretConstants;

/**
 * 
 */
public class TurretSubsystem extends SubsystemBase {
    // Advantagekit logging
    private final TurretModuleIO turretModuleIO;
    protected final TurretModuleIOInputsAutoLogged inputs;

    // Variables
    private double targetAngle;
    private boolean pidEnabled;

    private double[] kSVAs;
    private double[] kPIDs;

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
        this.targetAngle = 0.0;
        this.pidEnabled = false;

        // set shuffleboard entries if in tuning mode
        if (RobotConstants.TUNING_MODE) {
            kSVAs = TurretConstants.getSVAs();
            kPIDs = TurretConstants.getPIDs();

            setShuffleboard();
        }
    }

    @Override
    public void periodic() {
        // process hardware inputs
        turretModuleIO.updateInputs(inputs);
        Logger.processInputs("Turret", inputs);

        if (pidEnabled) {
            // update the turret voltage from pid controller & feedforward
            turretModuleIO.setAngle(targetAngle);
        }

        // update relative encoder if hall effect is triggered
        if (!turretModuleIO.getHallEffectValue()) {
            turretModuleIO.setPosition(TurretConstants.ANGLE_LIMIT);
        }

        // update pid values if in tuning mode
        if (RobotConstants.TUNING_MODE) {
            turretModuleIO.setFF(
                    sEntry.getDouble(kSVAs[0]),
                    vEntry.getDouble(kSVAs[1]),
                    aEntry.getDouble(kSVAs[2]));

            turretModuleIO.setPID(
                    pEntry.getDouble(kPIDs[0]),
                    iEntry.getDouble(kPIDs[1]),
                    dEntry.getDouble(kPIDs[2]));

            turretModuleIO.setConstraints(maxSpeedEntry.getDouble(TurretConstants.MAX_SPEED),
                    maxAccelEntry.getDouble(TurretConstants.MAX_ACCEL),
                    toleranceEntry.getDouble(TurretConstants.TOLERANCE));

            turretModuleIO.setAngle(angleEntry.getDouble(0.0));
        }

        Logger.recordOutput("Turret/CurrentAngle", getPosition());
        Logger.recordOutput("Turret/SetPointAngle", targetAngle);
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

    public boolean getHallEffectValue() {
        return turretModuleIO.getHallEffectValue();
    }

    /**
     * 
     */
    public Pose3d getPose(Pose2d robotPose) {
        Translation3d turretOffset = new Translation3d(TurretConstants.OFFSET_X, TurretConstants.OFFSET_Y,
                TurretConstants.OFFSET_Z);
        Rotation3d turretRotation = new Rotation3d(0.0,
                Math.toRadians(TurretConstants.verticalLaunchAngle),
                Math.toRadians(-getPosition()));
        Transform3d robotToTurret = new Transform3d(turretOffset, turretRotation);

        Pose3d robotPose3d = new Pose3d(robotPose);
        Pose3d turretPose = robotPose3d.plus(robotToTurret);

        return turretPose;
    }

    public double getPosition() {
        return turretModuleIO.getPosition();
    }

    public boolean isPIDEnabled() {
        return pidEnabled;
    }

    public void runTurret(double speed) {
        disablePID();
        turretModuleIO.set(speed);
    }

    public void setAngle(double targetAngle) {
        this.targetAngle = targetAngle;
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
        return Math.abs(getPosition() - targetAngle) <= TurretConstants.TOLERANCE;
    }

    /**
     * 
     */
    private void setShuffleboard() {
        tab = Shuffleboard.getTab("Turret");

        sEntry = tab.add("SET S", kSVAs[0]).getEntry();
        vEntry = tab.add("SET V", kSVAs[1]).getEntry();
        aEntry = tab.add("SET A", kSVAs[2]).getEntry();

        sEntry.setDouble(kSVAs[0]);
        vEntry.setDouble(kSVAs[1]);
        aEntry.setDouble(kSVAs[2]);

        pEntry = tab.add("SET P", kPIDs[0]).getEntry();
        iEntry = tab.add("SET I", kPIDs[1]).getEntry();
        dEntry = tab.add("SET D", kPIDs[2]).getEntry();

        pEntry.setDouble(kPIDs[0]);
        iEntry.setDouble(kPIDs[1]);
        dEntry.setDouble(kPIDs[2]);

        maxSpeedEntry = tab.add("SET MAXSPEED", TurretConstants.MAX_SPEED).getEntry();
        maxAccelEntry = tab.add("SET MAXACCEL", TurretConstants.MAX_ACCEL).getEntry();
        toleranceEntry = tab.add("SET TOLERANCE", TurretConstants.TOLERANCE).getEntry();

        maxSpeedEntry.setDouble(Constants.TurretConstants.MAX_SPEED);
        maxAccelEntry.setDouble(Constants.TurretConstants.MAX_ACCEL);
        toleranceEntry.setDouble(Constants.TurretConstants.TOLERANCE);

        angleEntry = tab.add("SET ANGLE", 0.0).getEntry();
        angleEntry.setDouble(0.0);

        enablePID();
    }
}
