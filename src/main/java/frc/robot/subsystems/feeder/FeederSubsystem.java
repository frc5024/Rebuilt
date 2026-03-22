// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.feeder;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.feederCommand;

/**
 * 
 */
public class FeederSubsystem extends SubsystemBase {
    // Advantagekit logging
    private final FeederModuleIO feederModuleIO;
    protected final FeederModuleIOInputsAutoLogged inputs;

    /**
     * 
     */
    public FeederSubsystem(FeederModuleIO feederModuleIO) {
        // set advantage kit IO logging
        this.feederModuleIO = feederModuleIO;
        this.inputs = new FeederModuleIOInputsAutoLogged();
    }

    @Override
    public void periodic() {
        // process hardware inputs
        feederModuleIO.updateInputs(inputs);
        Logger.processInputs("Feeder", inputs);

        Logger.recordOutput("Feeder/CurrentVelocityRPM", feederModuleIO.getVelocity());
    }

    public double getCurrentDrawAmps() {
        return feederModuleIO.getCurrentDrawAmps();
    }

    public double getPosition() {
        return feederModuleIO.getPosition();
    }

    public boolean isRunning() {
        return feederModuleIO.isRunning();
    }

    public void start() {
        feederModuleIO.start();
    }

    public void stop() {
        feederModuleIO.stop();
    }

    // TODO: remove and use stop instead
    public void setIdle() {
        feederModuleIO.set(0);
    }

    // TODO: remove and use start instead
    public void setFeederSpeed(double feederspeed) {
        feederModuleIO.set(feederspeed);
    }

    /**
     * Commands
     */
    public Command feederCommand() {
        return new feederCommand(this);
    }
}
