package frc.robot.subsystems.blower;

import java.util.LinkedList;
import java.util.NoSuchElementException;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.statemachine.StateMachine;
import frc.lib.statemachine.StateMetadata;

/**
 * 
 */
public class BlowerSubsystem extends SubsystemBase {
    private final String NAME = "Blower";

    /* Alerts */
    private final Alert disconnected = new Alert(NAME + " motor disconnected!", Alert.AlertType.kWarning);

    public static enum Action {
        IDLE, START, STOP
    }

    private final BlowerModuleIO blowerModuleIO;
    protected final BlowerModuleIOInputsAutoLogged inputs;
    protected final Timer stateTimer;

    private final StateMachine<Action> stateMachine;
    private final LinkedList<Action> actionQueue;

    /**
     * 
     */
    public BlowerSubsystem(BlowerModuleIO blowerModuleIO) {
        this.blowerModuleIO = blowerModuleIO;
        this.inputs = new BlowerModuleIOInputsAutoLogged();

        // Sets states for the arm, and what methods.
        this.stateMachine = new StateMachine<>(NAME);
        this.stateMachine.setDefaultState(Action.IDLE, this::handleIdle);
        this.stateMachine.addState(Action.START, this::handleStart);
        this.stateMachine.addState(Action.STOP, this::handleStop);

        this.actionQueue = new LinkedList<Action>();

        this.stateTimer = new Timer();
    }

    /**
     * 
     */
    public void addAction(Action action) {
        this.actionQueue.add(action);
    }

    /**
     * 
     */
    public Action getCurrentState() {
        return this.stateMachine.getCurrentState();
    }

    /**
     * 
     */
    protected void handleIdle(StateMetadata<Action> stateMetadata) {
        if (stateMetadata.isFirstRun()) {
            this.stateTimer.reset();
            this.stateTimer.start();
        }
    }

    /**
     * 
     */
    protected void handleStart(StateMetadata<Action> stateMetadata) {
        if (stateMetadata.isFirstRun()) {
            this.stateTimer.reset();
            this.stateTimer.start();
            this.blowerModuleIO.start();
        }
    }

    /**
     * 
     */
    protected void handleStop(StateMetadata<Action> stateMetadata) {
        if (stateMetadata.isFirstRun()) {
            this.blowerModuleIO.stop();
            this.stateTimer.stop();
            this.blowerModuleIO.stop();
        }
    }

    /**
     * 
     */
    private boolean isActionComplete() {
        switch (this.stateMachine.getCurrentState()) {
            default:
                return !this.stateTimer.isRunning();
        }
    }

    /**
     * 
     */
    public boolean isRunning() {
        return this.blowerModuleIO.isRunning();
    }

    /**
     * 
     */
    public void periodic() {
        this.stateMachine.update();

        this.blowerModuleIO.updateInputs(this.inputs);
        Logger.processInputs(this.NAME, this.inputs);
        double loadedVoltage = BatterySim.calculateDefaultBatteryLoadedVoltage(this.inputs.data.appliedVoltage());
        RoboRioSim.setVInVoltage(loadedVoltage);

        this.disconnected.set(!this.inputs.data.connected());

        // actions run for no longer than 3 seconds
        if (this.stateTimer.isRunning() && this.stateTimer.hasElapsed(15)) {
            this.stateTimer.stop();
        }

        if (isActionComplete()) {
            this.stateMachine.setState(Action.IDLE);
        }

        // Run any action in the queue
        if (this.stateMachine.getCurrentState() == Action.IDLE && this.actionQueue.size() > 0) {
            try {
                Action nextAction = this.actionQueue.removeFirst();
                this.stateMachine.setState(nextAction);
            } catch (NoSuchElementException e) {
            }
        }

        Logger.recordOutput("Subsystems/" + this.NAME + "/Current State", this.stateMachine.getCurrentState());
    }
}