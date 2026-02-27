package frc.robot.controllers;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.PathFinderAndFollowCommand;
import frc.robot.commands.runEverything;
import frc.robot.subsystems.climb.ClimbSubsystem;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.SwerveDriveSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;

/**
 * Define button commands when running the real robot
 */
public class ButtonBindings {
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
     * 
     */
    public ButtonBindings(SwerveDriveSubsystem swerveDriveSubsystem, ClimbSubsystem m_climb, FeederSubsystem m_feeder,
            HopperSubsystem m_hopper, IntakeSubsystem m_intake, ShooterSubsystem m_shooter, TurretSubsystem m_turret) {
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

        // driverController.b().whileTrue(m_hopper.SpinEntryCommand());
        commandXboxController.rightTrigger()
                .whileTrue(new InstantCommand(() -> swerveDriveSubsystem.isSlowMode = true));
        commandXboxController.rightTrigger().onFalse(new InstantCommand(() -> swerveDriveSubsystem.isSlowMode = false));
        commandXboxController.a().whileTrue(m_hopper.SpinCommand());
        commandXboxController.b().whileTrue(m_intake.OuttakeSpin());
        commandXboxController.leftTrigger().whileTrue(new runEverything(m_feeder, m_shooter, m_hopper));
        // commandXboxController.y().whileTrue(m_feeder.feedsdAerCommand());
        commandXboxController.y().onTrue(m_intake.RetractSpin());
        // commandXboxController.rightBumper().whileTrue(m_shooter.shooterCommand());
        commandXboxController.x()
                .onTrue(new InstantCommand(() -> swerveDriveSubsystem.setPose(swerveDriveSubsystem.getPose())));

        commandXboxController.rightBumper().onTrue((m_intake.ExtendSpin()));
        commandXboxController.rightBumper().whileTrue((m_intake.IntakeSpin()));

        // commandXboxController.leftTrigger().onTrue(m_intake.RetractSpin());
        commandXboxController.povUp().whileTrue(m_climb.climb());
        commandXboxController.povDown().whileTrue(m_climb.declimb());
        commandXboxController.povLeft().whileTrue(m_turret.stickRotation(-0.15));
        commandXboxController.povRight().whileTrue(m_turret.stickRotation(0.15));

        return commandXboxController;
    }

    /**
     * 
     */
    private CommandXboxController setOperatorBindingsController() {
        CommandXboxController commandXboxController = new CommandXboxController(OPERATOR_PORT);

        // For testing pathfinding
        commandXboxController.y().whileTrue(
                Commands.runOnce(() -> new PathFinderAndFollowCommand(swerveDriveSubsystem, "Pathfind Test Path")));

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
