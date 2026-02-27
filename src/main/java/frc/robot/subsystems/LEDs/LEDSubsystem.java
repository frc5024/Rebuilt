package frc.robot.subsystems.LEDs;

//Imports
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.leds.ILEDPreset;
import frc.lib.leds.LEDController;
import frc.lib.leds.LEDPreset;
import frc.robot.Constants;
import frc.robot.commands.LEDs.flashLEDS;
import frc.robot.commands.LEDs.persistLEDSCmd;
import frc.robot.commands.LEDs.setLEDS;
import frc.robot.containers.RobotContainer;

public class LEDSubsystem extends SubsystemBase {
    // Variables
    private static LEDSubsystem mInstance = null;
    private LEDController ledController;
    RobotContainer robotContainer;

    boolean visionMode = true;

    // Instance
    public static LEDSubsystem getInstance() {
        if (mInstance == null) {
            mInstance = new LEDSubsystem();
        }
        return mInstance;
    }

    // Constructor
    private LEDSubsystem() {
        ledController = new LEDController(Constants.LEDConstants.LEDStripID);
        // Sets which motor we are using, currently port 9

        // setDefaultCommand(new LEDDefaultCmd(this));
        set(LEDPreset.Solid.kBlack);
    }

    // Set the LEDs to be a colour
    public void set(ILEDPreset colour) {
        ledController.set(colour);
    }

    // Command Callers

    // Flash LEDs Command with one colour
    public Command flashCommand(ILEDPreset colour, int flashSeconds) {
        return new flashLEDS(this, colour, flashSeconds);
    }

    // Flash LEDs Command with two colours
    public Command flashCommand(ILEDPreset colour1, ILEDPreset colour2, int flashSeconds) {
        return new flashLEDS(this, colour1, colour2, flashSeconds);
    }

    // Set to colour Command
    public Command setCommand(ILEDPreset colour) {
        return new setLEDS(this, colour);
    }

    public Command persistCommand(ILEDPreset colour) {
        return new persistLEDSCmd(this, colour);
    }
}