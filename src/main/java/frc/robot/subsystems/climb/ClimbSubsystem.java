// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.ClimbCommands.ClimbCommand;
import frc.robot.commands.ClimbCommands.DeclimbCommand;
import frc.robot.commands.ClimbCommands.PauseclimbCommand;

/**
 * 
 */
public class ClimbSubsystem extends SubsystemBase {
    private final ClimbModuleIO climbModuleIO;
    protected final ClimbModuleIOInputsAutoLogged inputs;

    /** 
     * 
     */
    public ClimbSubsystem(ClimbModuleIO climbModuleIO) {
        this.climbModuleIO = climbModuleIO;
        this.inputs = new ClimbModuleIOInputsAutoLogged();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        climbModuleIO.updateInputs(inputs);
        Logger.processInputs("Climb", inputs);
    }

    public double getPosition() {
        return climbModuleIO.getPosition();
    }

    // Sets the speed of the climb motor to the inputted speel value
    public void setSpeed(Double speed) {
        climbModuleIO.set(speed);
    }

    // Calls ClimbCommand to set climb motor speed to climb speed
    public Command climb() {
        return new ClimbCommand(this);
    }

    // Calls DeclimbCommand to set climb motor speed to declimb speed
    public Command declimb() {
        return new DeclimbCommand(this);
    }

    // Calls PauseclimbCommand to set climb motor speed to stopped
    public Command dontdeclimb() {
        return new PauseclimbCommand(this);
    }
}
