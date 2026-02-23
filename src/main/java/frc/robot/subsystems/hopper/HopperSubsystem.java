package frc.robot.subsystems.hopper;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.Hopper.Spin;

/**
 * 
 */
public class HopperSubsystem extends SubsystemBase {
    private final HopperModuleIO hopperModuleIO;
    protected final HopperModuleIOInputsAutoLogged inputs;

    public double speed;
    public boolean direction;

    /**
     * 
     */
    public HopperSubsystem(HopperModuleIO hopperModuleIO) {
        this.hopperModuleIO = hopperModuleIO;
        this.inputs = new HopperModuleIOInputsAutoLogged();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        hopperModuleIO.updateInputs(inputs);
        Logger.processInputs("Hopper", inputs);
    }

    public void setSpeed(double speed) {
        hopperModuleIO.set(-speed);
    }

    public void setIdle() {
        hopperModuleIO.set(0);
    }

    public Command SpinCommand() {
        return new Spin(this);
    }

    // public Command SpinEntryCommand() {
    // return new Spin(this,
    // speedEntry.getDouble(Constants.HopperConstants.hopperSpeed));
    // }
}
