package frc.robot.util;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;

/**
 * Utility class for logging code execution times.
 */
public class LoggedTracer {
    // Variables
    private static final StringBuilder sb = new StringBuilder();
    private static double startTime = -1.0;

    /**
     * 
     */
    private LoggedTracer() {
    }

    /** Reset the clock. */
    public static void reset() {
        startTime = Timer.getFPGATimestamp();
    }

    /** Save the time elapsed since the last reset or record. */
    public static void record(String epochName) {
        double now = Timer.getFPGATimestamp();

        sb.setLength(0);
        sb.append("LoggedTracer/").append(epochName).append("MS");
        Logger.recordOutput(sb.toString(), (now - startTime) * 1000.0);

        startTime = now;
    }
}
