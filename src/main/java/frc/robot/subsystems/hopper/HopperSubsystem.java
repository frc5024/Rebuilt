package frc.robot.subsystems.hopper;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FeederConstants;
import frc.robot.Constants.HopperConstants;
import frc.robot.Constants.RobotConstants;

/**
 * 
 */
public class HopperSubsystem extends SubsystemBase {
    // Contants for detecting jam
    private final double JAM_CURRENT_THRESHOLD = 40.0; // Amps
    private final double JAM_TIMEOUT = 0.5; // Seconds
    private final double UNJAM_DURATION = 0.25; // Short pulse

    // Advantagekit logging
    private final HopperModuleIO hopperModuleIO;
    protected final HopperModuleIOInputsAutoLogged inputs;

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

    /**
     * 
     */
    public HopperSubsystem(HopperModuleIO hopperModuleIO) {
        // set advantage kit IO logging
        this.hopperModuleIO = hopperModuleIO;
        this.inputs = new HopperModuleIOInputsAutoLogged();

        this.targetRPM = 0;

        // setup for jam detection
        this.jamTimer = new Timer();
        this.unjamActionTimer = new Timer();
        this.isUnjamming = false;

        // set shuffleboard entries if in tuning mode
        if (RobotConstants.TUNING_MODE) {
            this.kSVAs = HopperConstants.getSVAs();
            this.kPIDs = HopperConstants.getPIDs();

            setShuffleboard();
        }
    }

    @Override
    public void periodic() {
        // process hardware inputs
        hopperModuleIO.updateInputs(inputs);
        Logger.processInputs("Hopper", inputs);

        // Set motor voltage based on sva and pid values
        hopperModuleIO.setVoltage(targetRPM);

        // check for ball jam
        checkAndHandleJam();

        // update pid values if in tuning mode
        if (RobotConstants.TUNING_MODE) {
            hopperModuleIO.setFF(
                    sEntry.getDouble(kSVAs[0]),
                    vEntry.getDouble(kSVAs[1]),
                    aEntry.getDouble(kSVAs[2]));

            hopperModuleIO.setPID(
                    pEntry.getDouble(kPIDs[0]),
                    iEntry.getDouble(kPIDs[1]),
                    dEntry.getDouble(kPIDs[2]));
        }

        Logger.recordOutput("Hopper/CurrentVelocityRPM", hopperModuleIO.getVelocity());
        Logger.recordOutput("Hopper/TargetRPM", targetRPM);
    }

    public double getCurrentDrawAmps() {
        return hopperModuleIO.getCurrentDrawAmps();
    }

    public double getPosition() {
        return hopperModuleIO.getPosition();
    }

    public boolean isRunning() {
        return hopperModuleIO.isRunning();
    }

    public void setVelocity(double targetRPM) {
        this.targetRPM = targetRPM;
    }

    public void stop() {
        hopperModuleIO.stop();
    }

    /**
     * 
     */
    private void checkAndHandleJam() {
        // check if we are currently unjamming
        if (isUnjamming) {
            if (unjamActionTimer.hasElapsed(UNJAM_DURATION)) {
                isUnjamming = true;
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
        if (jamTimer.hasElapsed(JAM_TIMEOUT)) {
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
        tab = Shuffleboard.getTab("Hopper");

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
    }
}
