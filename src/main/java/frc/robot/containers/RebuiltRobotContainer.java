package frc.robot.containers;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.VisionConstants;
import frc.robot.Robot;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.PathFinderAndFollowCommand;
import frc.robot.commands.TuningCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.blower.BlowerModuleIOTalonFX;
import frc.robot.subsystems.blower.BlowerSubsystem;
import frc.robot.subsystems.swervedrive.GyroIOPigeon2;
import frc.robot.subsystems.swervedrive.ModuleIOTalonFX;
import frc.robot.subsystems.swervedrive.SwerveDriveSubsystem;
import frc.robot.subsystems.vision.VisionIOLimelight;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * 
 */
public class RebuiltRobotContainer extends RobotContainer {

    // Controller
    private final CommandXboxController controller = new CommandXboxController(0);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RebuiltRobotContainer() {
        super();

        // Real robot, instantiate hardware IO implementations
        swerveDriveSubsystem = new SwerveDriveSubsystem(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight),
                (robotPose) -> {
                });
        visionSubsystem = new VisionSubsystem(
                swerveDriveSubsystem::addVisionMeasurement,
                new VisionIOLimelight(VisionConstants.camera0Name, swerveDriveSubsystem::getRotation),
                new VisionIOLimelight(VisionConstants.camera1Name, swerveDriveSubsystem::getRotation));

        blowerSubsystem = new BlowerSubsystem(new BlowerModuleIOTalonFX());

        configureAutoChooser();
        configureButtonBindings();
    }

    @Override
    protected void configureAutoChooser() {
        autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

        // Set up SysId routines
        autoChooser.addOption(
                "Drive Wheel Radius Characterization",
                TuningCommands.wheelRadiusCharacterization(swerveDriveSubsystem));
        autoChooser.addOption(
                "Drive Simple FF Characterization",
                TuningCommands.feedforwardCharacterization(swerveDriveSubsystem));
        autoChooser.addOption(
                "Drive SysId (Quasistatic Forward)",
                swerveDriveSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
                "Drive SysId (Quasistatic Reverse)",
                swerveDriveSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        autoChooser.addOption(
                "Drive SysId (Dynamic Forward)",
                swerveDriveSubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
                "Drive SysId (Dynamic Reverse)",
                swerveDriveSubsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    }

    // @Override
    // protected void configureButtonBindings() {
    // // Default command, normal field-relative drive
    // swerveDriveSubsystem.setDefaultCommand(
    // DriveCommands.joystickDrive(
    // swerveDriveSubsystem,
    // () -> -controller.getLeftY(),
    // () -> -controller.getLeftX(),
    // () -> -controller.getRightX()));

    // // Lock to 0° when A button is held
    // controller
    // .a()
    // .whileTrue(
    // DriveCommands.joystickDriveAtAngle(
    // swerveDriveSubsystem,
    // () -> -controller.getLeftY(),
    // () -> -controller.getLeftX(),
    // () -> new Rotation2d()));

    // // Switch to X pattern when X button is pressed
    // controller.x().onTrue(Commands.runOnce(swerveDriveSubsystem::stopWithX,
    // swerveDriveSubsystem));

    // // Reset gyro to 0° when B button is pressed
    // controller
    // .b()
    // .onTrue(
    // Commands.runOnce(
    // () -> swerveDriveSubsystem.setPose(
    // new Pose2d(swerveDriveSubsystem
    // .getPose()
    // .getTranslation(),
    // new Rotation2d())),
    // swerveDriveSubsystem)
    // .ignoringDisable(true));
    // controller.y().whileTrue(new PathFinderAndFollowCommand(swerveDriveSubsystem,
    // "Example Path"));

    // controller.rightTrigger().whileTrue(
    // new InstantCommand(() -> System.out.println("Right Trigger: " +
    // controller.getRightTriggerAxis())));
    // // Commands.runOnce(() -> blowerSubsystem.start(
    // // controller.getRightTriggerAxis())));

    // controller.leftTrigger().whileTrue(
    // new InstantCommand(() -> System.out.println("Left Trigger: " +
    // controller.getLeftTriggerAxis())));
    // // Commands.runOnce(() -> blowerSubsystem.start(
    // // controller.getLeftTriggerAxis() * -1)));
    // }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.get();
    }

    // Called periodically in simulation
    public void updateSimulation() {
    }

    public void onAllianceChanged(Alliance currentAlliance, int currentLocation) {
    }
}
