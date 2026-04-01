package frc.robot.subsystems.feeder;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.statemachine.StateMachineSubsystem;
import frc.robot.Constants.FeederConstants;
import frc.robot.Constants.RobotConstants;

/**
 * 
 */
public class FeederSubsystem extends StateMachineSubsystem {
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
    private GenericEntry rpmEntry;
    private GenericEntry simulateJamEntry;

    /**
     * 
     */
    public FeederSubsystem(FeederModuleIO feederModuleIO) {
        super("Feeder");

        // set advantage kit IO logging
        this.feederModuleIO = feederModuleIO;
        this.inputs = new FeederModuleIOInputsAutoLogged();

        // initial speed
        this.targetRPM = 0.0;

        // setup for jam detection
        this.jamTimer = new Timer();
        this.unjamActionTimer = new Timer();
        this.isUnjamming = false;
    }

    @Override
    public void periodic() {
        super.periodic();

        // process hardware inputs
        feederModuleIO.updateInputs(inputs);
        Logger.processInputs("Feeder", inputs);

        // Set motor voltage based on sva and pid values
        feederModuleIO.setVoltage(targetRPM);

        // check for ball jam
        checkAndHandleJam();

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
    @Override
    protected void setShuffleboard() {
        this.kSVAs = FeederConstants.getSVAs();
        this.kPIDs = FeederConstants.getPIDs();
    }

    @Override
    protected void setShuffleboardTab() {
        super.setShuffleboardTab();

        rpmEntry = tab.add("SET RPM", 0.0).getEntry();
        rpmEntry.setDouble(0.0);

        simulateJamEntry = tab.add("SIMULATE JAM", false)
                .withWidget("Toggle Switch")
                .getEntry();
    }

    @Override
    protected void setShuffleboardEntries() {
        // update pid values if in tuning mode
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

    /**
     * Overrides for SysId routines
     */
    @Override
    public double getFFCharacterizationVelocity() {
        return feederModuleIO.getFFCharacterizationVelocity();
    }

    @Override
    public void runCharacterization(double voltage) {
        feederModuleIO.runCharacterization(voltage);
    }
}
