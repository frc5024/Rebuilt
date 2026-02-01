package frc.robot.containers;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Robot;
import frc.robot.subsystems.swervedrive.SwerveDriveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * 
 */
abstract public class RobotContainer {
    /* Subsystems */
    protected SwerveDriveSubsystem swerveDriveSubsystem;
    protected VisionSubsystem visionSubsystem;

    /* Autonomous */
    protected LoggedDashboardChooser<Command> autoChooser;

    /* Controllers */
    CommandXboxController driverController;
    CommandXboxController operatorController;

    /**
     * 
     */
    protected void configureButtonBindings() {
        ButtonBindings buttonBindings = new ButtonBindings(swerveDriveSubsystem, visionSubsystem);
        driverController = buttonBindings.getDriverController();
        operatorController = buttonBindings.getOperatorController();
    }

    abstract protected void configureAutoChooser();

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.get();
    }

    // Methods used by simulation only

    // Creates the abstract method updateSimulation, which is called periodically
    // during simulation to update the state of the robot and its subsystems.
    public abstract void updateSimulation();

    // Creates the abstract method onAllianceChanged, which is called when the
    // alliance color or location changes, allowing the robot to adjust its behavior
    // accordingly.
    public abstract void onAllianceChanged(Alliance currentAlliance, int currentLocation);
}
