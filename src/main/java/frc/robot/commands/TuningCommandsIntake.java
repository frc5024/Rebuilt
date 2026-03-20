package frc.robot.commands;

import java.util.LinkedList;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class TuningCommandsIntake {
    private static final double FF_START_DELAY = 2.0; // Secs
    private static final double FF_RAMP_RATE = 0.1; // Volts/Sec

    /**
     * 
     */
    private TuningCommandsIntake() {
    }

    /**
     * Measures the velocity feedforward constants for the roller motor.
     *
     * <p>
     * This command should only be used in voltage control mode.
     */
    public static Command feedforwardCharacterization(IntakeSubsystem intake) {
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
                            intake.runCharacterization(0.0);
                        },
                        intake)
                        .withTimeout(FF_START_DELAY),

                // Start timer
                Commands.runOnce(timer::restart),

                // Accelerate and gather data
                Commands.run(
                        () -> {
                            double voltage = timer.get() * FF_RAMP_RATE;
                            intake.runCharacterization(voltage);
                            velocitySamples.add(intake.getFFCharacterizationVelocity());
                            voltageSamples.add(voltage);
                        },
                        intake)

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

                                    Logger.recordOutput("Intake/FF_Characterization/kS", kS);
                                    Logger.recordOutput("Intake/FF_Characterization/kV", kV);
                                }));
    }
}
