package frc.robot.util;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;

/**
 * 
 */
public class MatchSim {
    // 2026 REBUILT Timing (Seconds remaining in match)
    private static final double AUTO_END = 135.0;
    private static final double TRANSITION_END = 125.0; // 10s Transition
    private static final double SHIFT_1_END = 100.0; // 25s per Shift
    private static final double SHIFT_2_END = 75.0;
    private static final double SHIFT_3_END = 50.0;
    private static final double SHIFT_4_END = 25.0; // End Game starts at 25s

    /**
     * 
     */
    public static void update() {
        if (!RobotBase.isSimulation()) {
            return;
        }

        double matchTime = Timer.getMatchTime();
        boolean hubActive = true;

        // Logic for the 2026 Shift System
        if (matchTime > AUTO_END) {
            hubActive = true; // Auto: Both Active
        } else if (matchTime > TRANSITION_END) {
            hubActive = true; // Transition: Both Active
        } else if (matchTime > SHIFT_1_END) {
            hubActive = false; // Shift 1: Inactive (Assuming your alliance won Auto)
        } else if (matchTime > SHIFT_2_END) {
            hubActive = true; // Shift 2: Active
        } else if (matchTime > SHIFT_3_END) {
            hubActive = false; // Shift 3: Inactive
        } else if (matchTime > SHIFT_4_END) {
            hubActive = true; // Shift 4: Active
        } else {
            hubActive = true; // End Game: Both Active
        }

        // Log the state so your Intake/Shooter knows if it's "legal" to score
        Logger.recordOutput("MatchSim/HubActive", hubActive);

        // Simulating the FMS Game Data String for 2026
        String gameData = hubActive ? "ACTIVE" : "INACTIVE";
        DriverStationSim.setGameSpecificMessage(gameData);
    }
}
