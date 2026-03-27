package frc.robot.commands.LEDs;

//Imports
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.leds.ILEDPreset;
import frc.robot.subsystems.LEDs.LEDSubsystem;

public class fadeLEDS extends Command {
    // Variables
    private LEDSubsystem leds; // LED Subsystem for calling
    private ILEDPreset colour1; // Colour #1 we wish to set colour to initially (See constructor)
    private ILEDPreset colour2; // Colour #2 we wish to fade into
    private int flashSeconds; // Amount of time we wish to flash for (IN SECONDS), used to keep track of total
                              // time elapsed
    private int flashMiliseconds; // Amount of time we wish to flash for (IN MILLISECONDS), used to keep track of
                                  // how many times we have flashed

    private Timer totalTime = new Timer(); // Timer to check total amount of time elapsed since the start of the command
    private Timer timer = new Timer(); // Timer to check time elapsed
    private int flashCount = 0; // A counter to ensure amount of flashes

    // Constructor for two colours
    public fadeLEDS(LEDSubsystem leds, ILEDPreset colour1, ILEDPreset colour2, int flashSeconds) {
        this.leds = leds; // Subsystem set
        this.colour1 = colour1; // Colour 1 set
        this.colour2 = colour2; // Colour 2 set
        this.flashSeconds = flashSeconds; // Total time set
        flashMiliseconds = flashSeconds * 100; // Conversion from seconds to milliseconds
    }

    @Override
    public void initialize() {
        flashCount = 0;// Resets count just to be safe
        timer.reset();// Resets timer
        timer.restart();// Starts timer
        totalTime.reset();
        totalTime.restart();
    }

    @Override
    public void execute() {
        if (flashCount < flashMiliseconds) {
            // the .hasElapsed function checks how much time has elapsed since the start of
            // the command, if not restarted will keep increasing and the function will not
            // operate correctly
        }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // After certain time has elapsed stop command and reset to default
        if (totalTime.hasElapsed(flashSeconds)) {
            return true;
        }
        return false;
    }
}
