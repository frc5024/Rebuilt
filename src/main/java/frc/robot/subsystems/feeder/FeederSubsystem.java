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
    private final FeederModuleIO feederModuleIO;
    protected final FeederModuleIOInputsAutoLogged inputs;

    /**
     * 
     */
    public FeederSubsystem(FeederModuleIO feederModuleIO) {
        this.feederModuleIO = feederModuleIO;
        this.inputs = new FeederModuleIOInputsAutoLogged();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        feederModuleIO.updateInputs(inputs);
        Logger.processInputs("Feeder", inputs);
    }

    public void setIdle() {
        feederModuleIO.set(0);
    }

    public void setFeederSpeed(double feederspeed) {
        feederModuleIO.set(feederspeed);
    }

    public Command feederCommand() {
        return new feederCommand(this);
    }

    // public Command jammedCommand() {
    // return new feederJamCommand(this);
    // }
}
