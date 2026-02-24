package frc.robot.controllers;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.PathFinderAndFollowCommand;
import frc.robot.subsystems.climb.ClimbSubsystem;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.SwerveDriveSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;

/**
 * 
 */
public class ButtonsBindingsSim {
    private final int DRIVER_PORT = 0;
    private final int OPERATOR_PORT = 1;
    private final int TEST_PORT = 2;

    /* Controllers */
    private final CommandXboxController driverController;
    private final CommandXboxController operatorController;
    private final CommandXboxController testController;

    /* Subsystems */
    private final SwerveDriveSubsystem swerveDriveSubsystem;
    private ClimbSubsystem m_climb;
    private FeederSubsystem m_feeder;
    private HopperSubsystem m_hopper;
    private IntakeSubsystem m_intake;
    private ShooterSubsystem m_shooter;
    private TurretSubsystem m_turret;

    /**
     * Define button commands when running the simulator
     */
    public ButtonsBindingsSim(SwerveDriveSubsystem swerveDriveSubsystem, ClimbSubsystem m_climb,
            FeederSubsystem m_feeder, HopperSubsystem m_hopper, IntakeSubsystem m_intake, ShooterSubsystem m_shooter,
            TurretSubsystem m_turret) {
        this.swerveDriveSubsystem = swerveDriveSubsystem;
        this.m_climb = m_climb;
        this.m_feeder = m_feeder;
        this.m_hopper = m_hopper;
        this.m_intake = m_intake;
        this.m_shooter = m_shooter;
        this.m_turret = m_turret;

        this.driverController = setDriverBindingsController();
        this.operatorController = setOperatorBindingsController();

        // Set this to whichever button bindings you want to test
        // this.buttonTestController = setLEDTestBindingsController();
        this.testController = setTuningBindings();
    }

    /**
     * 
     */
    private CommandXboxController setDriverBindingsController() {
        CommandXboxController commandXboxController = new CommandXboxController(DRIVER_PORT);
        // Default command, normal field-relative drive
        swerveDriveSubsystem.setDefaultCommand(
                DriveCommands.joystickDrive(
                        swerveDriveSubsystem,
                        () -> -commandXboxController.getLeftY(),
                        () -> -commandXboxController.getLeftX(),
                        () -> -commandXboxController.getRightX()));

        // Lock to 0° when A button is held
        commandXboxController.a().whileTrue(
                DriveCommands.joystickDriveAtAngle(
                        swerveDriveSubsystem,
                        () -> -commandXboxController.getLeftY(),
                        () -> -commandXboxController.getLeftX(),
                        () -> new Rotation2d()));

        // Reset gyro to 0° when B button is pressed
        commandXboxController.b().onTrue(
                Commands.runOnce(
                        () -> swerveDriveSubsystem.setPose(
                                new Pose2d(swerveDriveSubsystem.getPose().getTranslation(), new Rotation2d())),
                        swerveDriveSubsystem)
                        .ignoringDisable(true));

        // Switch to X pattern when X button is pressed
        commandXboxController.x().onTrue(Commands.runOnce(swerveDriveSubsystem::stopWithX, swerveDriveSubsystem));

        commandXboxController.y().whileTrue(new PathFinderAndFollowCommand(swerveDriveSubsystem, "Example Path"));

        commandXboxController.povUp().whileTrue(m_climb.climb());
        commandXboxController.povDown().whileTrue(m_climb.declimb());

        return commandXboxController;
    }

    /**
     * 
     */
    private CommandXboxController setOperatorBindingsController() {
        CommandXboxController commandXboxController = new CommandXboxController(OPERATOR_PORT);

        commandXboxController.a().whileTrue(m_hopper.SpinCommand());

        return commandXboxController;
    }

    /**
     * 
     */
    private CommandXboxController setLEDTestBindingsController() {
        CommandXboxController commandXboxController = new CommandXboxController(TEST_PORT);

        return commandXboxController;
    }

    /**
     * 
     */
    private CommandXboxController setTuningBindings() {
        CommandXboxController commandXboxController = new CommandXboxController(TEST_PORT);

        return commandXboxController;
    }

    /**
     * 
     */
    public CommandXboxController getDriverController() {
        return this.driverController;
    }

    /**
     * 
     */
    public CommandXboxController getOperatorController() {
        return this.operatorController;
    }

    /**
     * 
     */
    public CommandXboxController getTestController() {
        return this.testController;
    }
}
