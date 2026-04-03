package frc.lib.statemachine;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.LinkedList;
import java.util.List;
import java.util.NoSuchElementException;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.RobotConstants;
import frc.robot.util.LoggedTracer;

/**
 * 
 */
abstract public class StateMachineSubsystem extends SubsystemBase {
    // Constants
    private static final double FF_START_DELAY = 2.0; // Secs
    private static final double FF_RAMP_RATE = 0.1; // Volts/Sec

    // Track subsystem by name
    protected final String name;

    // Alerts
    protected final Alert disconnected;

    // States and State Machine
    public static enum Action {
        IDLE, START, STOP
    }

    protected final StateMachine<Action> stateMachine;
    protected final LinkedList<Action> actionQueue;
    protected final Timer stateTimer;
    protected final StringBuilder sb;

    // SysId
    protected final SysIdRoutine sysIdRoutine;

    // PID & SVA
    protected double[] kSVAs = { 0.0, 0.0, 0.0 };
    protected double[] kPIDs = { 0.0, 0.0, 0.0 };

    // Shuffleboard entries
    protected ShuffleboardTab tab;

    protected GenericEntry sEntry;
    protected GenericEntry vEntry;
    protected GenericEntry aEntry;

    protected GenericEntry pEntry;
    protected GenericEntry iEntry;
    protected GenericEntry dEntry;

    protected GenericEntry angleEntry;
    protected GenericEntry rpmEntry;

    /**
     * 
     */
    public StateMachineSubsystem(String name) {
        this.name = name;
        this.disconnected = new Alert(name + " motor disconnected!", Alert.AlertType.kWarning);

        // Sets states for the climb arm, and what methods.
        this.stateMachine = new StateMachine<>(name);
        this.stateMachine.setDefaultState(Action.IDLE, this::handleIdle);
        this.stateMachine.addState(Action.START, this::handleStart);
        this.stateMachine.addState(Action.STOP, this::handleStop);

        this.actionQueue = new LinkedList<Action>();

        this.stateTimer = new Timer();
        this.sb = new StringBuilder();

        // Configure SysId
        SysIdRoutine.Config config = new SysIdRoutine.Config(
                Volts.of(1.0).per(Seconds),
                Volts.of(7.0),
                Seconds.of(10.0),
                (state) -> {
                    Logger.recordOutput("SysId/" + name + "/State", state.toString());
                });
        SysIdRoutine.Mechanism mechanism = new SysIdRoutine.Mechanism(
                (voltage) -> runCharacterization(voltage.in(Volts)),
                null,
                this);
        sysIdRoutine = new SysIdRoutine(config, mechanism);

        // set shuffleboard entries if in tuning mode
        if (RobotConstants.TUNING_MODE) {
            setShuffleboard();
            setShuffleboardTab();
        }
    }

    /**
     * 
     */
    public void addAction(Action action) {
        actionQueue.add(action);
    }

    /**
     * 
     */
    protected Action getCurrentState() {
        return stateMachine.getCurrentState();
    }

    /**
     * 
     */
    protected void handleIdle(StateMetadata<Action> stateMetadata) {
        if (stateMetadata.isFirstRun()) {
            stateTimer.reset();
            stateTimer.start();
        }
    }

    /**
     * 
     */
    protected void handleStart(StateMetadata<Action> stateMetadata) {
        if (stateMetadata.isFirstRun()) {
            stateTimer.reset();
            stateTimer.start();
        }
    }

    /**
     * 
     */
    protected void handleStop(StateMetadata<Action> stateMetadata) {
        if (stateMetadata.isFirstRun()) {
            stateTimer.reset();
            stateTimer.start();
        }
    }

    /**
     * 
     */
    protected boolean isActionComplete() {
        switch (this.stateMachine.getCurrentState()) {
            case START:
                return !this.stateTimer.isRunning();
            case STOP:
                return !this.stateTimer.isRunning();
            default:
                return !this.stateTimer.isRunning();
        }
    }

    @Override
    public void periodic() {
        this.stateMachine.update();

        // actions run for no longer than 3 seconds
        if (this.stateTimer.isRunning() && this.stateTimer.hasElapsed(3)) {
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

        // set shuffleboard entries if in tuning mode
        if (RobotConstants.TUNING_MODE) {
            setShuffleboardEntries();
        }

        Command currentCommand = getCurrentCommand();
        Command defaultCommand = getDefaultCommand();

        sb.setLength(0);
        sb.append("Subsystems/").append(name).append("/CurrentState");
        Logger.recordOutput(sb.toString(), stateMachine.getCurrentState());

        sb.setLength(0);
        sb.append("Subsystems/").append(name).append("/CurrentCommand");
        Logger.recordOutput(sb.toString(), currentCommand != null ? currentCommand.getName() : "");

        sb.setLength(0);
        sb.append("Subsystems/").append(name).append("/DefaultCommand");
        Logger.recordOutput(sb.toString(), defaultCommand != null ? defaultCommand.getName() : "");

        // Record cycle time
        LoggedTracer.record(name);
    }

    /**
     * Routines for running SysId tuning
     */
    abstract public double getFFCharacterizationVelocity();

    /**
     * Measures the velocity feedforward constants for the drive motors.
     *
     * <p>
     * This command should only be used in voltage control mode.
     */
    public Command feedforwardCharacterization() {
        List<Double> velocitySamples = new LinkedList<>();
        List<Double> voltageSamples = new LinkedList<>();
        Timer timer = new Timer();

        return Commands.sequence(
                // Reset data
                Commands.runOnce(
                        () -> {
                            velocitySamples.clear();
                            voltageSamples.clear();
                        }),

                // Allow modules to orient
                Commands.run(
                        () -> {
                            runCharacterization(0.0);
                        },
                        this)
                        .withTimeout(FF_START_DELAY),

                // Start timer
                Commands.runOnce(timer::restart),

                // Accelerate and gather data
                Commands.run(
                        () -> {
                            double voltage = timer.get() * FF_RAMP_RATE;
                            runCharacterization(voltage);
                            velocitySamples.add(getFFCharacterizationVelocity());
                            voltageSamples.add(voltage);
                        },
                        this)

                        // When cancelled, calculate and print results
                        .finallyDo(
                                () -> {
                                    int n = velocitySamples.size();
                                    double sumX = 0.0;
                                    double sumY = 0.0;
                                    double sumXY = 0.0;
                                    double sumX2 = 0.0;
                                    for (int i = 0; i < n; i++) {
                                        sumX += velocitySamples.get(i);
                                        sumY += voltageSamples.get(i);
                                        sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                                        sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                                    }
                                    double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                                    double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                                    Logger.recordOutput("SysId/" + name + "/FF_Characterization/kS", kS);
                                    Logger.recordOutput("SysId/" + name + "/FF_Characterization/kV", kV);
                                }));
    }

    abstract public void runCharacterization(double output);

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return run(() -> runCharacterization(0.0))
                .withTimeout(1.0)
                .andThen(sysIdRoutine.quasistatic(direction));
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysIdRoutine.dynamic(direction));
    }

    /**
     * 
     */
    abstract protected void setShuffleboard();

    protected void setShuffleboardTab() {
        tab = Shuffleboard.getTab(name);

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

        angleEntry = tab.add("SET ANGLE", 0.0).getEntry();
        angleEntry.setDouble(0.0);

        rpmEntry = tab.add("SET RPM", 0.0).getEntry();
        rpmEntry.setDouble(0.0);
    }

    abstract protected void setShuffleboardEntries();
}
