package frc.robot.controllers;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommands;
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
 * Define button commands when running the real robot
 */
public class ButtonBindings {
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

    /**
     * 
     */
    public ButtonBindings(SwerveDriveSubsystem swerveDriveSubsystem, ClimbSubsystem climbSubsystem,
            FeederSubsystem feederSubsystem,
            HopperSubsystem hopperSubsystem, IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem,
            TurretSubsystem turretSubsystem) {
        this.swerveDriveSubsystem = swerveDriveSubsystem;
        this.climbSubsystem = climbSubsystem;
        this.feederSubsystem = feederSubsystem;
        this.hopperSubsystem = hopperSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.turretSubsystem = turretSubsystem;

        this.driverController = setDriverBindingsController();
        this.operatorController = setOperatorBindingsController();

        // Set this to whichever button bindpings you want to test
        // this.buttonTestController = setLEDTestBindingsController();
        this.tuningController = setTuningBindings();
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

        commandXboxController.y().onTrue(intakeSubsystem.RetractArmCommand());

        commandXboxController.leftTrigger()
                .whileTrue(
                        new ShootCommand(swerveDriveSubsystem, hopperSubsystem, feederSubsystem, shooterSubsystem,
                                0.0));

        commandXboxController.rightTrigger()
                .whileTrue(new InstantCommand(() -> swerveDriveSubsystem.setSlowMode(true)));
        commandXboxController.rightTrigger().onFalse(new InstantCommand(() -> swerveDriveSubsystem.setSlowMode(false)));

        commandXboxController.rightBumper().onTrue((intakeSubsystem.ExtendArmCommand()));
        commandXboxController.rightBumper().whileTrue(intakeSubsystem.IntakeCommand());

        commandXboxController.back().whileTrue(intakeSubsystem.OuttakeCommand());

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

        commandXboxController.rightBumper()
                .onTrue(Commands.runOnce(() -> turretSubsystem.setAngle(30.0), turretSubsystem));
        commandXboxController.leftBumper()
                .onTrue(Commands.runOnce(() -> turretSubsystem.setAngle(-30.0), turretSubsystem));

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
