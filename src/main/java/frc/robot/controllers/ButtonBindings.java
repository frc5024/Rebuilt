package frc.robot.controllers;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.PathFinderAndFollowCommand;
import frc.robot.commands.distanceShooterCommand;
import frc.robot.commands.runEverything;
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
        commandXboxController.leftTrigger()
                .whileTrue(Commands.parallel(new runEverything(m_feeder, m_shooter, m_hopper),
                        new distanceShooterCommand(m_shooter, swerveDriveSubsystem)));
        commandXboxController.rightTrigger()
                .whileTrue(new InstantCommand(() -> swerveDriveSubsystem.isSlowMode = true));
        commandXboxController.rightTrigger().onFalse(new InstantCommand(() -> swerveDriveSubsystem.isSlowMode = false));
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

        // commandXboxController.b().whileTrue(m_intake.OuttakeCommand());
        // commandXboxController.b().whileTrue(m_intake.retractArmPIDCommand());
        // commandXboxController.leftTrigger().whileTrue( new runEverything(m_feeder,
        // m_shooter, m_hopper));
        // commandXboxController.y().whileTrue(m_feeder.feedsdAerCommand());
        commandXboxController.y().onTrue(m_intake.RetractArmCommand());
        // commandXboxController.rightBumper().whileTrue(m_shooter.shooterCommand());
        // commandXboxController.x().whileTrue(new distanceShooterCommand(m_shooter,
        // swerveDriveSubsystem));
        // commandXboxController.x()
        // .onTrue(new InstantCommand(() ->
        // swerveDriveSubsystem.setPose(swerveDriveSubsystem.getPose())));
        // commandXboxController.x().whileTrue(m_intake.extendArmPIDCommand());
        commandXboxController.rightBumper().onTrue((m_intake.ExtendArmCommand()));
        commandXboxController.rightBumper().whileTrue((m_intake.IntakeCommand()));

        // commandXboxController.leftTrigger().onTrue(m_intake.RetractSpin());
        commandXboxController.povUp().whileTrue(m_climb.extend());
        commandXboxController.povDown().whileTrue(m_climb.contract());
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
