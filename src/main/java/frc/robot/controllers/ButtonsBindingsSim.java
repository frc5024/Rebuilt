package frc.robot.controllers;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.FeederConstants;
import frc.robot.Constants.HopperConstants;
import frc.robot.commands.ClearTheWallCommand;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.DriveNearestTrenchCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.StickRotationCommand;
import frc.robot.commands.turretSweepCommand;
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

        // commandXboxController.a().whileTrue(m_hopper.SpinCommand());
        commandXboxController.a().whileTrue(
                DriveCommands.joystickDriveAtAngle(
                        swerveDriveSubsystem,
                        () -> -commandXboxController.getLeftY(),
                        () -> -commandXboxController.getLeftX(),
                        () -> {
                            Pose2d robotPose = swerveDriveSubsystem.getPose();
                            double robotX = robotPose.getX();
                            double robotY = robotPose.getY();

                            Pose2d hubPose = GameUtil.getHubPose();
                            double hubX = hubPose.getX();
                            double hubY = hubPose.getY();

                            double angleToHub = Math.atan2(hubY - robotY, hubX - robotX);
                            return Rotation2d.fromRadians(angleToHub);
                        }));

        commandXboxController.x().whileTrue(
                new DriveNearestTrenchCommand(() -> swerveDriveSubsystem.getPose()));

        commandXboxController.y().whileTrue(
                new ClearTheWallCommand(swerveDriveSubsystem, intakeSubsystem, shooterSubsystem, hopperSubsystem,
                        feederSubsystem));

        commandXboxController.rightBumper().onTrue(intakeSubsystem.ExtendArmCommand());
        commandXboxController.leftBumper().onTrue(intakeSubsystem.RetractArmCommand());
        // commandXboxController.rightBumper().whileTrue(m_intake.IntakeCommand());
        commandXboxController.back().onTrue(
                Commands.runOnce(() -> {
                    if (!intakeToggleState) {
                        CommandScheduler.getInstance().schedule(intakeSubsystem.OuttakeCommand());
                    } else {
                        CommandScheduler.getInstance().schedule(intakeSubsystem.IntakeCommand());
                    }

                    intakeToggleState = !intakeToggleState;
                }, intakeSubsystem));

        // commandXboxController.leftTrigger()
        // .whileTrue(
        // Commands.parallel(
        // new runEverything(m_feeder, m_shooter, m_hopper),
        // new distanceShooterCommand(m_shooter, swerveDriveSubsystem)));
        commandXboxController.leftTrigger()
                .whileTrue(new ShootCommand(swerveDriveSubsystem, shooterSubsystem, hopperSubsystem, feederSubsystem,
                        () -> swerveDriveSubsystem.getPose()));

        commandXboxController.rightTrigger()
                .whileTrue(
                        new InstantCommand(() -> swerveDriveSubsystem.setSlowMode(true)));
        commandXboxController.rightTrigger()
                .onFalse(
                        new InstantCommand(() -> swerveDriveSubsystem.setSlowMode(false)));

        commandXboxController.povUp().whileTrue(climbSubsystem.extendclimb());
        commandXboxController.povDown().whileTrue(climbSubsystem.contractclimb());

        commandXboxController.povLeft().onTrue(new turretSweepCommand(turretSubsystem));
        commandXboxController.povRight().whileTrue(new StickRotationCommand(turretSubsystem, -0.1));

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

        commandXboxController.a().onTrue(Commands.runOnce(() -> hopperSubsystem.setVelocity(0.0),
                hopperSubsystem));
        commandXboxController.y()
                .onTrue(Commands.runOnce(() -> hopperSubsystem.setVelocity(HopperConstants.RPM), hopperSubsystem));

        commandXboxController.a().onTrue(Commands.runOnce(() -> feederSubsystem.setVelocity(0.0), feederSubsystem));
        commandXboxController.y()
                .onTrue(Commands.runOnce(() -> feederSubsystem.setVelocity(FeederConstants.RPM), feederSubsystem));

        // commandXboxController.y().onTrue(Commands.runOnce(() ->
        // m_intake.intakeRoller(), m_intake));
        // commandXboxController.a().onTrue(Commands.runOnce(() ->
        // m_intake.stopRoller(), m_intake));

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
