package frc.robot.controllers;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.distanceShooterCommand;
import frc.robot.commands.runEverything;
import frc.robot.commands.turretToggle;
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

        // Set this to whichever button bindpings you want to test
        // this.buttonTestController = setLEDTestBindingsController();
        this.testController = setTuningBindings();
    }

    /**
     * Driver Controller Button Bindings
     */
    private CommandXboxController setDriverBindingsController() {
        CommandXboxController controller = new CommandXboxController(DRIVER_PORT);

        // ========== DEFAULT COMMAND ==========
        swerveDriveSubsystem.setDefaultCommand(
                DriveCommands.joystickDrive(
                        swerveDriveSubsystem,
                        () -> -controller.getLeftY(),
                        () -> -controller.getLeftX(),
                        () -> -controller.getRightX()));

        // ========== SHOOTING (TRIGGERS) ==========
        // Left Trigger: Shoot with arm jostling (settles fuel)
        controller.leftTrigger()
                .whileTrue(Commands.parallel(
                        new runEverything(m_feeder, m_shooter, m_hopper),
                        new distanceShooterCommand(m_shooter, swerveDriveSubsystem),
                        m_intake.jostleArmCommand()));

        // controller.leftTrigger().whileTrue(m_intake.jostleArmCommand());

        // Left Bumper: Shoot without arm jostling (for static shots)
        controller.leftBumper()
                .whileTrue(Commands.parallel(
                        new runEverything(m_feeder, m_shooter, m_hopper),
                        new distanceShooterCommand(m_shooter, swerveDriveSubsystem)));

        // ========== DRIVE MODE (RIGHT TRIGGERS + START) ==========
        // Right Trigger: Enable slow mode
        controller.rightTrigger()
                .whileTrue(new StartEndCommand(
                        () -> swerveDriveSubsystem.isSlowMode = true,
                        () -> swerveDriveSubsystem.isSlowMode = false));

        controller.leftTrigger()
                .whileTrue(new StartEndCommand(
                        () -> swerveDriveSubsystem.isSlowMode = true,
                        () -> swerveDriveSubsystem.isSlowMode = false));

        controller.leftBumper()
                .whileTrue(new StartEndCommand(
                        () -> swerveDriveSubsystem.isSlowMode = true,
                        () -> swerveDriveSubsystem.isSlowMode = false));

        // Start Button: Toggle turret on/off
        controller.start().onTrue(new turretToggle(m_turret));

        // ========== FACE BUTTONS (DRIVE DIRECTION) ==========
        // A Button: Aim robot toward hub
        controller.a().whileTrue(
                DriveCommands.joystickDriveAtAngle(
                        swerveDriveSubsystem,
                        () -> -controller.getLeftY(),
                        () -> -controller.getLeftX(),
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

        // X Button: Spin turret to aim at hub (with ballistic lead)
        // controller.x().whileTrue(
        // new spinToHubCommand(m_turret,
        // () -> swerveDriveSubsystem.getPose(),
        // () -> swerveDriveSubsystem.getChassisSpeeds()));

        // Y Button: Retract intake arm
        controller.y().onTrue(m_intake.RetractArmCommand());

        // ========== INTAKE (RIGHT BUMPER + BACK) ==========
        // Right Bumper: Extend arm and run adaptive intake (speed-based RPM)
        controller.rightBumper().onTrue(m_intake.ExtendArmCommand());
        controller.rightBumper().whileTrue(
                m_intake.adaptiveIntakeCommand(() -> swerveDriveSubsystem.getChassisSpeeds()));

        // Back Button: Outtake (reverse intake)
        controller.back().whileTrue(m_intake.outtakePIDCommand());

        // ========== CLIMB (D-PAD UP/DOWN) ==========
        controller.povUp().whileTrue(m_climb.extendclimb());
        controller.povDown().whileTrue(m_climb.contractclimb());

        // ========== TURRET TUNING (D-PAD LEFT/RIGHT) ==========
        // controller.povLeft().whileTrue(new zeroTurret(m_turret));
        // controller.povRight().whileTrue(new StickRotationCommand(m_turret, -0.1));

        return controller;
    }

    /**
     * Operator Controller Button Bindings
     */
    private CommandXboxController setOperatorBindingsController() {
        CommandXboxController commandXboxController = new CommandXboxController(OPERATOR_PORT);

        // Reset climb on X button
        commandXboxController.x().whileTrue(m_climb.resetClimb());
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

        commandXboxController.x().whileTrue(new InstantCommand(() -> m_turret.setAngle(90), m_turret));
        // commandXboxController.b().whileTrue(new InstantCommand(() ->
        // m_turret.setAngle(-90), m_turret));

        Logger.recordOutput("Turret/Bindings", true);

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
