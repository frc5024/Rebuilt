package frc.robot.subsystems.hopper;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.Hopper.Spin;

/**
 * 
 */
public class HopperSubsystem extends SubsystemBase {
    // Advantagekit logging
    private final HopperModuleIO hopperModuleIO;
    protected final HopperModuleIOInputsAutoLogged inputs;

    // Variables
    public double speed;
    public boolean direction;

    /**
     * 
     */
    public HopperSubsystem(HopperModuleIO hopperModuleIO) {
        // set advantage kit IO logging
        this.hopperModuleIO = hopperModuleIO;
        this.inputs = new HopperModuleIOInputsAutoLogged();
    }

    @Override
    public void periodic() {
        // process hardware inputs
        hopperModuleIO.updateInputs(inputs);
        Logger.processInputs("Hopper", inputs);

        Logger.recordOutput("Hopper/CurrentVelocityRPM", hopperModuleIO.getVelocity());
    }

    public double getCurrentDrawAmps() {
        return hopperModuleIO.getCurrentDrawAmps();
    }

    public double getPosition() {
        return hopperModuleIO.getPosition();
    }

    public void start() {
        hopperModuleIO.start();
    }

    public void stop() {
        hopperModuleIO.stop();
    }

    // TODO: remove and use start instead
    public void setSpeed(double speed) {
        hopperModuleIO.set(-speed);
    }

    // TODO: remove and use stop instead
    public void setIdle() {
        hopperModuleIO.set(0);
    }

    /**
     * Commands
     */
    public Command SpinCommand() {
        return new Spin(this);
    }
}
