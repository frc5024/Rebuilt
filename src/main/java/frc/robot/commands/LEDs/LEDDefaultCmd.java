// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LEDs;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.leds.LEDPreset;
import frc.robot.subsystems.LEDs.LEDSubsystem;

public class LEDDefaultCmd extends Command {

    private final LEDSubsystem s_LED;
    // Timer shooterTimer = new Timer();
    // Timer intakeTimer = new Timer();

    public LEDDefaultCmd(LEDSubsystem s_LED) {

        this.s_LED = s_LED;

        addRequirements(s_LED);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        s_LED.set(LEDPreset.Solid.kBlack);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
