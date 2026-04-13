package frc.robot.commands.Intake;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.intakeConstants;
import frc.robot.subsystems.intake.IntakeSubsystem;

/**
 * Jostles the intake arm up and down to help settle fuel into the hopper
 * - Retracts arm using RETRACTING_SPEED voltage
 * - Extends arm using EXTENDING_SPEED voltage (stops when limit switch hits)
 * - Repeats while command is running
 * - Stops when left trigger is released
 */
public class JostleArmCommand extends Command {
    private IntakeSubsystem s_Intake;
    private Timer cycleTimer;

    // Configuration
    static ShuffleboardTab tab = Shuffleboard.getTab("intakeMotor");
    static GenericEntry retractTimeEntry = tab.add("JOSTLE RETRACT TIME (ms)", 300).getEntry();
    static GenericEntry extendTimeEntry = tab.add("JOSTLE EXTEND TIME (ms)", 400).getEntry();

    public JostleArmCommand(IntakeSubsystem s_Intake) {
        this.s_Intake = s_Intake;
        this.cycleTimer = new Timer();
        addRequirements(s_Intake);
    }

    @Override
    public void initialize() {
        cycleTimer.reset();
        cycleTimer.start();
    }

    @Override
    public void execute() {
        double retractTime = retractTimeEntry.getDouble(300) / 1000.0; // Convert ms to seconds
        double extendTime = extendTimeEntry.getDouble(400) / 1000.0; // Convert ms to seconds

        double cycleTime = retractTime + extendTime;
        double elapsedInCycle = cycleTimer.get() % cycleTime;

        if (elapsedInCycle < retractTime) {
            // Retracting phase - use the normal retracting voltage
            s_Intake.setArmSpeed(intakeConstants.RETRACTING_SPEED);
        } else {
            // Extending phase - use the normal extending voltage
            // Stops automatically when limit switch (extended) is hit
            s_Intake.setArmSpeed(intakeConstants.EXTENDING_SPEED);
        }
    }

    @Override
    public void end(boolean interrupted) {
        s_Intake.setArmSpeed(0.0);
        cycleTimer.stop();
    }

    @Override
    public boolean isFinished() {
        // This command runs until manually cancelled (trigger released)
        return false;
    }
}
