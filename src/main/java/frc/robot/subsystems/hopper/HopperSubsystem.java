package frc.robot.subsystems.hopper;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.statemachine.StateMachineSubsystem;
import frc.robot.Constants.HopperConstants;
import frc.robot.Constants.RobotConstants;

/**
 * 
 */
public class HopperSubsystem extends StateMachineSubsystem {
    // Contants for detecting jam
    private final double JAM_CURRENT_THRESHOLD = 40.0; // Amps
    private final double JAM_TIMEOUT = 0.5; // Seconds
    private final double UNJAM_DURATION = 0.25; // Short pulse

    // Advantagekit logging
    private final HopperModuleIO hopperModuleIO;
    protected final HopperModuleIOInputsAutoLogged inputs;

    // Variables
    private double targetRPM;

    // Used for checking if jammed
    private Timer jamTimer;
    private Timer unjamActionTimer;
    private boolean isUnjamming;

    // Shuffleboard entries
    private GenericEntry simulateJamEntry;

    /**
     * 
     */
    public HopperSubsystem(HopperModuleIO hopperModuleIO) {
        super("Hopper");

        // set advantage kit IO logging
        this.hopperModuleIO = hopperModuleIO;
        this.inputs = new HopperModuleIOInputsAutoLogged();

        // initial speed
        this.targetRPM = 0;

        // setup for jam detection
        this.jamTimer = new Timer();
        this.unjamActionTimer = new Timer();
        this.isUnjamming = false;
    }

    @Override
    public void periodic() {
        super.periodic();

        // process hardware inputs
        hopperModuleIO.updateInputs(inputs);
        Logger.processInputs("Hopper", inputs);

        // Set motor voltage based on sva and pid values
        hopperModuleIO.setVoltage(targetRPM);

        // check for ball jam
        checkAndHandleJam();

        Logger.recordOutput("Subsystems/Hopper/CurrentRPM", hopperModuleIO.getVelocity());
        Logger.recordOutput("Subsystems/Hopper/TargetRPM", targetRPM);
    }

    public double getCurrentDrawAmps() {
        if (RobotConstants.TUNING_MODE) {
            if (simulateJamEntry.getBoolean(false) && !isUnjamming) {
                return JAM_CURRENT_THRESHOLD + 10.0;
            }
        }

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
                isUnjamming = false;
                unjamActionTimer.stop();
                targetRPM = 0.0;
                stop();
            } else {
                targetRPM = HopperConstants.UNJAM_RPM;
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
        this.kSVAs = HopperConstants.getSVAs();
        this.kPIDs = HopperConstants.getPIDs();
    }

    @Override
    protected void setShuffleboardTab() {
        super.setShuffleboardTab();

        simulateJamEntry = tab.add("SIMULATE JAM", false)
                .withWidget("Toggle Switch")
                .getEntry();
    }

    @Override
    protected void setShuffleboardEntries() {
        hopperModuleIO.setFF(
                sEntry.getDouble(kSVAs[0]),
                vEntry.getDouble(kSVAs[1]),
                aEntry.getDouble(kSVAs[2]));

        hopperModuleIO.setPID(
                pEntry.getDouble(kPIDs[0]),
                iEntry.getDouble(kPIDs[1]),
                dEntry.getDouble(kPIDs[2]));

        setVelocity(rpmEntry.getDouble(0.0));
    }

    /**
     * Overrides for SysId routines
     */
    @Override
    public double getFFCharacterizationVelocity() {
        return hopperModuleIO.getFFCharacterizationVelocity();
    }

    @Override
    public void runCharacterization(double voltage) {
        hopperModuleIO.runCharacterization(voltage);
    }
}
