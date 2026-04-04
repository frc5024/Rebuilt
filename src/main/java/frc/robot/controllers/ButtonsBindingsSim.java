package frc.robot.controllers;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ClimbConstants;
import frc.robot.commands.ClearTheWallCommand;
import frc.robot.commands.ClimbToBarCommand;
import frc.robot.commands.CollectAndHerdCommand;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.DriveNearestTrenchCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.subsystems.climb.ClimbSubsystem;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.SwerveDriveSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.util.GameUtil;

/**
 * 
 */
public class ButtonsBindingsSim {
    // Constants
    private final int DRIVER_PORT = 0;
    private final int OPERATOR_PORT = 1;
    private final int TEST_PORT = 2;

    /* Controllers */
    private final CommandXboxController driverController;
    private final CommandXboxController operatorController;
    private final CommandXboxController tuningController;

    /* Subsystems */
    private final SwerveDriveSubsystem swerveDriveSubsystem;
    private ClimbSubsystem climbSubsystem;
    private FeederSubsystem feederSubsystem;
    private HopperSubsystem hopperSubsystem;
    private IntakeSubsystem intakeSubsystem;
    private ShooterSubsystem shooterSubsystem;
    private TurretSubsystem turretSubsystem;

    // Variables
    private boolean intakeToggleState;

    /**
     * Define button commands when running the simulator
     */
    public ButtonsBindingsSim(SwerveDriveSubsystem swerveDriveSubsystem, ClimbSubsystem climbSubsystem,
            FeederSubsystem feederSubsystem, HopperSubsystem hopperSubsystem, IntakeSubsystem intakeSubsystem,
            ShooterSubsystem shooterSubsystem, TurretSubsystem turretSubsystem) {
        this.swerveDriveSubsystem = swerveDriveSubsystem;
        this.climbSubsystem = climbSubsystem;
        this.feederSubsystem = feederSubsystem;
        this.hopperSubsystem = hopperSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.turretSubsystem = turretSubsystem;

        this.driverController = setDriverBindingsController();
        this.operatorController = setOperatorBindingsController();

        // Set this to whichever button bindings you want to test
        // this.buttonTestController = setLEDTestBindingsController();
        this.tuningController = setTuningBindings();

        this.intakeToggleState = false;
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

        commandXboxController.a()
                .whileTrue(
                        new ClimbToBarCommand(climbSubsystem, () -> swerveDriveSubsystem.getPose()));

        commandXboxController.b()
                .whileTrue(
                        new CollectAndHerdCommand(intakeSubsystem, swerveDriveSubsystem::getPose));

        commandXboxController.x()
                .whileTrue(
                        new DriveNearestTrenchCommand(() -> swerveDriveSubsystem.getPose()));

        commandXboxController.y()
                .whileTrue(
                        new ClearTheWallCommand(swerveDriveSubsystem, intakeSubsystem, shooterSubsystem,
                                hopperSubsystem,
                                feederSubsystem));

        commandXboxController.rightBumper()
                .onTrue(
                        Commands.sequence(
                                Commands.runOnce(() -> intakeSubsystem.intakeRoller(), intakeSubsystem),
                                Commands.runOnce(() -> intakeSubsystem.extendArm(), intakeSubsystem)));

        commandXboxController.leftBumper()
                .onTrue(
                        Commands.sequence(
                                Commands.runOnce(() -> intakeSubsystem.stopRoller(), intakeSubsystem),
                                Commands.runOnce(() -> intakeSubsystem.retractArm(), intakeSubsystem)));

        commandXboxController.povUp()
                .whileTrue(
                        Commands.runOnce(() -> climbSubsystem.setPosition(ClimbConstants.EXTEND_LENGTH_INCHES, false),
                                climbSubsystem));

        commandXboxController.povDown()
                .whileTrue(
                        Commands.runOnce(() -> climbSubsystem.setPosition(ClimbConstants.ZERO_LENGTH_INCHES, false),
                                climbSubsystem));

        commandXboxController.povLeft()
                .whileTrue(
                        DriveCommands.joystickDriveAtAngle(
                                swerveDriveSubsystem,
                                () -> -commandXboxController.getLeftY(),
                                () -> -commandXboxController.getLeftX(),
                                () -> {
                                    Pose2d robotPose = swerveDriveSubsystem.getPose();
                                    Pose2d targetPose = GameUtil.getTargetPose(robotPose);
                                    return targetPose.getTranslation().minus(robotPose.getTranslation())
                                            .getAngle();
                                }));

        commandXboxController.back()
                .onTrue(
                        Commands.runOnce(() -> {
                            if (!intakeToggleState) {
                                CommandScheduler.getInstance().schedule(intakeSubsystem.OuttakeCommand());
                            } else {
                                CommandScheduler.getInstance().schedule(intakeSubsystem.IntakeCommand());
                            }

                            intakeToggleState = !intakeToggleState;
                        }, intakeSubsystem));

        commandXboxController.rightTrigger()
                .whileTrue(
                        Commands.runOnce(() -> swerveDriveSubsystem.setSlowMode(true)));
        commandXboxController.rightTrigger()
                .onFalse(
                        Commands.runOnce(() -> swerveDriveSubsystem.setSlowMode(false)));

        commandXboxController.leftTrigger()
                .whileTrue(
                        new ShootCommand(swerveDriveSubsystem, shooterSubsystem, hopperSubsystem, feederSubsystem));

        return commandXboxController;
    }

    /**
     * 
     */
    private CommandXboxController setOperatorBindingsController() {
        CommandXboxController commandXboxController = new CommandXboxController(OPERATOR_PORT);

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

        // commandXboxController.b().onTrue(Commands.runOnce(() ->
        // m_turret.setAngle(90.0), m_turret));
        // commandXboxController.x().onTrue(Commands.runOnce(() ->
        // m_turret.setAngle(-90.0), m_turret));

        // commandXboxController.b().onTrue(Commands.runOnce(() -> m_intake.extendArm(),
        // m_intake));
        // commandXboxController.x().onTrue(Commands.runOnce(() ->
        // m_intake.retractArm(), m_intake));

        // commandXboxController.b().onTrue(Commands.runOnce(() ->
        // m_shooter.setVelocity(0), m_shooter));
        // commandXboxController.x().onTrue(Commands.runOnce(() ->
        // m_shooter.setVelocity(2800), m_shooter));

        // commandXboxController.leftTrigger()
        // .whileTrue(new ShootCommand(swerveDriveSubsystem, m_shooter, m_hopper,
        // m_feeder,
        // () -> swerveDriveSubsystem.getPose()));

        // commandXboxController.a().onTrue(Commands.runOnce(() ->
        // hopperSubsystem.setVelocity(0.0),
        // hopperSubsystem));
        // commandXboxController.y()
        // .onTrue(Commands.runOnce(() ->
        // hopperSubsystem.setVelocity(HopperConstants.RPM), hopperSubsystem));

        // commandXboxController.a().onTrue(Commands.runOnce(() ->
        // feederSubsystem.setVelocity(0.0), feederSubsystem));
        // commandXboxController.y()
        // .onTrue(Commands.runOnce(() ->
        // feederSubsystem.setVelocity(FeederConstants.RPM), feederSubsystem));

        commandXboxController.y().onTrue(Commands.runOnce(() -> intakeSubsystem.intakeRoller(), intakeSubsystem));
        commandXboxController.a().onTrue(Commands.runOnce(() -> intakeSubsystem.stopRoller(), intakeSubsystem));

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
    public CommandXboxController getTuningController() {
        return this.tuningController;
    }
}
