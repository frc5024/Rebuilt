// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.ClimbCommands.ClimbCommand;
import frc.robot.commands.ClimbCommands.ContractCommand;
import frc.robot.commands.ClimbCommands.DeclimbCommand;
import frc.robot.commands.ClimbCommands.ExtendCommand;
import frc.robot.commands.ClimbCommands.PauseclimbCommand;

/**
 * 
 */
public class ClimbSubsystem extends SubsystemBase {
    // Advantagekit logging
    private final ClimbModuleIO climbModuleIO;
    protected final ClimbModuleIOInputsAutoLogged inputs;

    /** 
    * 
    */
    public ClimbSubsystem(ClimbModuleIO climbModuleIO) {
        // set advantage kit IO logging
        this.climbModuleIO = climbModuleIO;
        this.inputs = new ClimbModuleIOInputsAutoLogged();
    }

    @Override
    public void periodic() {
        // process hardware inputs
        climbModuleIO.updateInputs(inputs);
        Logger.processInputs("Climb", inputs);

        SmartDashboard.putNumber("Climb Position Rads", inputs.data.positionRads());
    }

    public double getCurrentDrawAmps() {
        return climbModuleIO.getCurrentDrawAmps();
    }

    public double getPosition() {
        return climbModuleIO.getPosition();
    }

    public void zeroPosition() {
        climbModuleIO.zeroPosition();
    }

    // Sets the speed of the climb motor to the inputted speel value
    public void setSpeed(Double speed) {
        climbModuleIO.set(speed);
    }

    public Double value() {
        return inputs.data.positionRads();
    }

    // Calls ClimbCommand to set climb motor speed to climb speed (Same direction as
    // contract speed)
    public Command climb() {
        return new ClimbCommand(this);
    }

    // Calls DeclimbCommand to set climb motor speed to declimb speed (Same
    // direction as extend speed)
    public Command declimb() {
        return new DeclimbCommand(this);
    }

    // Calls PauseclimbCommand to set climb motor speed to stopped
    public Command dontdeclimb() {
        return new PauseclimbCommand(this);
    }

    // Calls ExtendCommand to set climb motor speed to extend speed (Same direction
    // as declimb speed)
    public Command extendclimb() {
        return new ExtendCommand(this);
    }

    // Calls ContractCommand to set climb motor speed to contract speed (Same
    // direction as climb speed)
    public Command contractclimb() {
        return new ContractCommand(this);
    }
}
