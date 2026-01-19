package frc.robot.containers;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Robot;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.PathFinderAndFollowCommand;
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
    public RobotContainer() {
    }

    abstract protected void configureAutoChooser();

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
     * it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    protected void configureButtonBindings() {
        driverController = new CommandXboxController(0);

        // Default command, normal field-relative drive
        swerveDriveSubsystem.setDefaultCommand(
                DriveCommands.joystickDrive(
                        swerveDriveSubsystem,
                        () -> -driverController.getLeftY(),
                        () -> -driverController.getLeftX(),
                        () -> -driverController.getRightX()));

        // Lock to 0° when A button is held
        driverController
                .a()
                .whileTrue(
                        DriveCommands.joystickDriveAtAngle(
                                swerveDriveSubsystem,
                                () -> -driverController.getLeftY(),
                                () -> -driverController.getLeftX(),
                                () -> new Rotation2d()));

        // Switch to X pattern when X button is pressed
        driverController.x().onTrue(Commands.runOnce(swerveDriveSubsystem::stopWithX, swerveDriveSubsystem));

        // Reset gyro to 0° when B button is pressed
        driverController
                .b()
                .onTrue(
                        Commands.runOnce(
                                () -> swerveDriveSubsystem.setPose(
                                        new Pose2d(swerveDriveSubsystem.getPose().getTranslation(), new Rotation2d())),
                                swerveDriveSubsystem)
                                .ignoringDisable(true));
        driverController.y().whileTrue(new PathFinderAndFollowCommand(swerveDriveSubsystem, "Example Path"));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.get();
    }

    // Methods used by simulation only
    public abstract void updateSimulation();

    public abstract void onAllianceChanged(Alliance currentAlliance, int currentLocation);
}
