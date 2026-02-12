package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.blower.BlowerSubsystem;
import frc.robot.util.LoggedTunableNumber;

/**
 * 
 */
public class BlowerTuningCommand extends Command {
    private final BlowerSubsystem blowerSubsystem;

    private static final LoggedTunableNumber blowerkP = new LoggedTunableNumber("Blower/kP");
    private static final LoggedTunableNumber blowerkD = new LoggedTunableNumber("Blower/kD");

    private static final LoggedTunableNumber blowerkS = new LoggedTunableNumber("Blower/kS");
    private static final LoggedTunableNumber blowerkV = new LoggedTunableNumber("Blower/kV");
    private static final LoggedTunableNumber blowerkG = new LoggedTunableNumber("Blower/kG");

    private static final LoggedTunableNumber mmCruiseVelocity = new LoggedTunableNumber("MotionMagic/CruiseVelocity");
    private static final LoggedTunableNumber mmAcceleration = new LoggedTunableNumber("MotionMagic/Acceleration");
    private static final LoggedTunableNumber mmJerk = new LoggedTunableNumber("MotionMagic/Jerk");

    static {
        blowerkP.initDefault(4.8);
        blowerkD.initDefault(0.1);

        blowerkS.initDefault(0.25);
        blowerkV.initDefault(0.12);
        blowerkG.initDefault(0.01);

        mmCruiseVelocity.initDefault(80.0);
        mmAcceleration.initDefault(160.0);
        mmJerk.initDefault(1600.0);
    }

    /**
     * 
     */
    public BlowerTuningCommand(BlowerSubsystem blowerSubsystem) {
        this.blowerSubsystem = blowerSubsystem;

        addRequirements(blowerSubsystem);
    }

    @Override
    public void initialize() {
        Logger.recordOutput("Commands/Active Command", this.getName());
    }

    @Override
    public void execute() {
        // Update from tunable numbers
        if (blowerkP.hasChanged(hashCode()) || blowerkD.hasChanged(hashCode())) {
            this.blowerSubsystem.setPID(blowerkP.get(), blowerkD.get(), blowerkS.get(), blowerkV.get(), blowerkG.get());
        }

        // Update from tunable numbers
        if (blowerkS.hasChanged(hashCode()) || blowerkV.hasChanged(hashCode()) || blowerkG.hasChanged(hashCode())) {
            this.blowerSubsystem.setPID(blowerkP.get(), blowerkD.get(), blowerkS.get(), blowerkV.get(), blowerkG.get());
        }

        // Update from tunable numbers
        if (mmCruiseVelocity.hasChanged(hashCode()) || mmAcceleration.hasChanged(hashCode())
                || mmJerk.hasChanged(hashCode())) {
            this.blowerSubsystem.setMM(mmCruiseVelocity.get(), mmAcceleration.get(), mmJerk.get());
        }
    }

    @Override
    public void end(boolean interrupted) {
        this.blowerSubsystem.addAction(BlowerSubsystem.Action.STOP);
        Logger.recordOutput("Commands/Active Command", "");
    }
}
