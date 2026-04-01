package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.GenericEntry;
import frc.lib.statemachine.StateMachineSubsystem;
import frc.robot.Constants;
import frc.robot.Constants.TurretConstants;

/**
 * 
 */
public class TurretSubsystem extends StateMachineSubsystem {
    // Advantagekit logging
    private final TurretModuleIO turretModuleIO;
    protected final TurretModuleIOInputsAutoLogged inputs;

    // Variables
    private double targetAngle;
    private boolean pidEnabled;

    // Shuffleboard entries
    private GenericEntry maxSpeedEntry;
    private GenericEntry maxAccelEntry;
    private GenericEntry toleranceEntry;

    /**
     * 
     */
    public TurretSubsystem(TurretModuleIO turretModuleIO) {
        super("Turret");

        // set advantage kit IO logging
        this.turretModuleIO = turretModuleIO;
        this.inputs = new TurretModuleIOInputsAutoLogged();
        this.targetAngle = 0.0;
        this.pidEnabled = false;
    }

    @Override
    public void periodic() {
        super.periodic();

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

        Logger.recordOutput("Turret/CurrentAngle", getPosition());
        Logger.recordOutput("Turret/TargetAngle", targetAngle);
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
                Math.toRadians(-turretModuleIO.getPosition()));
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
    @Override
    protected void setShuffleboard() {
        kSVAs = TurretConstants.getSVAs();
        kPIDs = TurretConstants.getPIDs();
    }

    @Override
    protected void setShuffleboardTab() {
        super.setShuffleboardTab();

        maxSpeedEntry = tab.add("SET MAXSPEED", TurretConstants.MAX_SPEED).getEntry();
        maxAccelEntry = tab.add("SET MAXACCEL", TurretConstants.MAX_ACCEL).getEntry();
        toleranceEntry = tab.add("SET TOLERANCE", TurretConstants.TOLERANCE).getEntry();

        maxSpeedEntry.setDouble(Constants.TurretConstants.MAX_SPEED);
        maxAccelEntry.setDouble(Constants.TurretConstants.MAX_ACCEL);
        toleranceEntry.setDouble(Constants.TurretConstants.TOLERANCE);

        enablePID();
    }

    @Override
    protected void setShuffleboardEntries() {
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

        setAngle(angleEntry.getDouble(0.0));
    }

    /**
     * Overrides for SysId routines
     */
    @Override
    public double getFFCharacterizationVelocity() {
        return turretModuleIO.getFFCharacterizationVelocity();
    }

    @Override
    public void runCharacterization(double voltage) {
        turretModuleIO.runCharacterization(voltage);
    }
}
