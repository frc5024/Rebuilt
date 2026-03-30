package frc.robot.subsystems.feeder;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FeederConstants;
import frc.robot.Constants.RobotConstants;

/**
 * 
 */
public class FeederSubsystem extends SubsystemBase {
    // Contants for detecting jam
    private final double JAM_CURRENT_THRESHOLD = 40.0; // Amps
    private final double JAM_TIMEOUT = 0.5; // Seconds
    private final double UNJAM_DURATION = 0.25; // Short pulse

    // Advantagekit logging
    private final FeederModuleIO feederModuleIO;
    protected final FeederModuleIOInputsAutoLogged inputs;

    // Variables
    private double targetRPM;
    private double[] kSVAs;
    private double[] kPIDs;

    // Used for checking if jammed
    private Timer jamTimer;
    private Timer unjamActionTimer;
    private boolean isUnjamming;

    // Shuffleboard entries
    private ShuffleboardTab tab;

    private GenericEntry sEntry;
    private GenericEntry vEntry;
    private GenericEntry aEntry;

    private GenericEntry pEntry;
    private GenericEntry dEntry;
    private GenericEntry iEntry;

    private GenericEntry rpmEntry;
    private GenericEntry simulateJamEntry;

    /**
     * 
     */
    public FeederSubsystem(FeederModuleIO feederModuleIO) {
        // set advantage kit IO logging
        this.feederModuleIO = feederModuleIO;
        this.inputs = new FeederModuleIOInputsAutoLogged();

        // initial speed
        this.targetRPM = 0.0;

        // setup for jam detection
        this.jamTimer = new Timer();
        this.unjamActionTimer = new Timer();
        this.isUnjamming = false;

        // set shuffleboard entries if in tuning mode
        if (RobotConstants.TUNING_MODE) {
            this.kSVAs = FeederConstants.getSVAs();
            this.kPIDs = FeederConstants.getPIDs();

            setShuffleboard();
        }
    }

    @Override
    public void periodic() {
        // process hardware inputs
        feederModuleIO.updateInputs(inputs);
        Logger.processInputs("Feeder", inputs);

        // Set motor voltage based on sva and pid values
        feederModuleIO.setVoltage(targetRPM);

        // check for ball jam
        checkAndHandleJam();

        // update pid values if in tuning mode
        if (RobotConstants.TUNING_MODE) {
            feederModuleIO.setFF(
                    sEntry.getDouble(kSVAs[0]),
                    vEntry.getDouble(kSVAs[1]),
                    aEntry.getDouble(kSVAs[2]));

            feederModuleIO.setPID(
                    pEntry.getDouble(kPIDs[0]),
                    iEntry.getDouble(kPIDs[1]),
                    dEntry.getDouble(kPIDs[2]));

            feederModuleIO.setVoltage(rpmEntry.getDouble(0.0));
        }

        Logger.recordOutput("Feeder/CurrentVelocityRPM", feederModuleIO.getVelocity());
        Logger.recordOutput("Feeder/TargetRPM", targetRPM);
    }

    public double getCurrentDrawAmps() {
        if (RobotConstants.TUNING_MODE) {
            if (simulateJamEntry.getBoolean(false) && !isUnjamming) {
                return JAM_CURRENT_THRESHOLD + 10.0;
            }
        }

        return feederModuleIO.getCurrentDrawAmps();
    }

    public double getPosition() {
        return feederModuleIO.getPosition();
    }

    public boolean isRunning() {
        return feederModuleIO.isRunning();
    }

    public void setVelocity(double targetRPM) {
        this.targetRPM = targetRPM;
    }

    public void stop() {
        feederModuleIO.stop();
    }

    /**
     * 
     */
    private void checkAndHandleJam() {
        // check if we are currently unjamming
        if (isUnjamming) {
            if (unjamActionTimer.hasElapsed(UNJAM_DURATION)) {
                isUnjamming = false;
                unjamActionTimer.stop();
                targetRPM = 0.0;
                stop();
            } else {
                targetRPM = FeederConstants.UNJAM_RPM;
                return;
            }
        }

        // check if current is high and motor should be moving
        if (getCurrentDrawAmps() > JAM_CURRENT_THRESHOLD && targetRPM != 0.0) {
            jamTimer.start();
        } else {
            jamTimer.stop();
            jamTimer.reset();
        }

        // unjam if threshold is held long enough
        if (jamTimer.isRunning() && jamTimer.hasElapsed(JAM_TIMEOUT)) {
            isUnjamming = true;
            jamTimer.reset();
            unjamActionTimer.reset();
            unjamActionTimer.start();
        }
    }

    /**
     * 
     */
    private void setShuffleboard() {
        tab = Shuffleboard.getTab("Feeder");

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

        rpmEntry = tab.add("SET RPM", 0.0).getEntry();
        rpmEntry.setDouble(0.0);

        simulateJamEntry = tab.add("SIMULATE JAM", false)
                .withWidget("Toggle Switch")
                .getEntry();
    }
}
