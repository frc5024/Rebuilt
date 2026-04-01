package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.Logger;

import frc.lib.statemachine.StateMachineSubsystem;
import frc.robot.Constants.ClimbConstants;

/**
 * 
 */
public class ClimbSubsystem extends StateMachineSubsystem {
    // Advantagekit logging
    private final ClimbModuleIO climbModuleIO;
    protected final ClimbModuleIOInputsAutoLogged inputs;

    /** 
    * 
    */
    public ClimbSubsystem(ClimbModuleIO climbModuleIO) {
        super("Climb");

        // set advantage kit IO logging
        this.climbModuleIO = climbModuleIO;
        this.inputs = new ClimbModuleIOInputsAutoLogged();
    }

    @Override
    public void periodic() {
        super.periodic();

        // process hardware inputs
        climbModuleIO.updateInputs(inputs);
        Logger.processInputs("Climb", inputs);
    }

    public double getCurrentDrawAmps() {
        return climbModuleIO.getCurrentDrawAmps();
    }

    public double getPosition() {
        return climbModuleIO.getPosition();
    }

    /**
     * 
     */
    @Override
    protected void setShuffleboard() {
        this.kSVAs = ClimbConstants.getSVAs();
        this.kPIDs = ClimbConstants.getPIDs();
    }

    @Override
    protected void setShuffleboardEntries() {
        // update pid values if in tuning mode
        climbModuleIO.setFF(
                sEntry.getDouble(kSVAs[0]),
                vEntry.getDouble(kSVAs[1]),
                aEntry.getDouble(kSVAs[2]));

        climbModuleIO.setPID(
                pEntry.getDouble(kPIDs[0]),
                iEntry.getDouble(kPIDs[1]),
                dEntry.getDouble(kPIDs[2]));
    }

    /**
     * Overrides for SysId routines
     */
    @Override
    public double getFFCharacterizationVelocity() {
        return climbModuleIO.getFFCharacterizationVelocity();
    }

    @Override
    public void runCharacterization(double voltage) {
        climbModuleIO.runCharacterization(voltage);
    }
}
