package frc.robot.containers;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.FuelCellConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.LockTurretOnTarget;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.TuningCommandsDrive;
import frc.robot.generated.TunerConstants;
import frc.robot.mechanisms.MechanismVisualizer;
import frc.robot.subsystems.climb.ClimbModuleIOTalonFX;
import frc.robot.subsystems.climb.ClimbSubsystem;
import frc.robot.subsystems.feeder.FeederModuleIOSparkFlexRelativeEncoder;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.hopper.HopperModuleIOSparkMaxRelativeEncoder;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.intake.ArmModuleIOSparkMaxRelativeEncoder;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.RollerModuleIOSparkFlexClosedLoopController;
import frc.robot.subsystems.shooter.ShooterModuleIOSparkFlexClosedLoopController;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.GyroModuleIOPigeon2;
import frc.robot.subsystems.swervedrive.SwerveDriveSubsystem;
import frc.robot.subsystems.swervedrive.SwerveModuleIOTalonFXMotionMagic;
import frc.robot.subsystems.turret.TurretModuleIOSparkMaxRelativeEncoder;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionSubsystem;

/**
 * 
 */
public class RebuiltRobotContainer extends RobotContainer {
    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RebuiltRobotContainer() {
        super();

        // used to visually simulate intake, hopper, turret, climb
        this.mechanismVisualizer = new MechanismVisualizer();

        // Real robot, instantiate hardware IO implementations
        this.swerveDriveSubsystem = new SwerveDriveSubsystem(
                new GyroModuleIOPigeon2(),
                new SwerveModuleIOTalonFXMotionMagic(TunerConstants.FrontLeft),
                new SwerveModuleIOTalonFXMotionMagic(TunerConstants.FrontRight),
                new SwerveModuleIOTalonFXMotionMagic(TunerConstants.BackLeft),
                new SwerveModuleIOTalonFXMotionMagic(TunerConstants.BackRight),
                (robotPose) -> {
                });

        this.visionSubsystem = new VisionSubsystem(
                swerveDriveSubsystem::addVisionMeasurement,
                swerveDriveSubsystem::getChassisSpeeds,
                new VisionIOLimelight(VisionConstants.frontCamera, swerveDriveSubsystem::getRotation),
                new VisionIOLimelight(VisionConstants.rearCamera, swerveDriveSubsystem::getRotation));

        this.climbSubsystem = new ClimbSubsystem(new ClimbModuleIOTalonFX());
        this.feederSubsystem = new FeederSubsystem(new FeederModuleIOSparkFlexRelativeEncoder());
        this.hopperSubsystem = new HopperSubsystem(new HopperModuleIOSparkMaxRelativeEncoder());
        this.intakeSubsystem = new IntakeSubsystem(new ArmModuleIOSparkMaxRelativeEncoder(),
                new RollerModuleIOSparkFlexClosedLoopController());
        this.shooterSubsystem = new ShooterSubsystem(new ShooterModuleIOSparkFlexClosedLoopController());
        this.turretSubsystem = new TurretSubsystem(new TurretModuleIOSparkMaxRelativeEncoder());

        if (!RobotConstants.TUNING_MODE) {
            this.climbSubsystem
                    .setDefaultCommand(Commands.runOnce(() -> climbSubsystem.holdPosition(), climbSubsystem));
            this.shooterSubsystem
                    .setDefaultCommand(Commands.runOnce(
                            () -> shooterSubsystem.setVelocity(ShooterConstants.IDLE_SPEED_RPM), shooterSubsystem));
            this.turretSubsystem
                    .setDefaultCommand(new LockTurretOnTarget(turretSubsystem,
                            () -> swerveDriveSubsystem.getPose(),
                            () -> swerveDriveSubsystem.getChassisSpeeds(),
                            () -> shooterSubsystem.getTangentialVelocity(),
                            shooterSubsystem::addDistanceMeasurement));
        }

        turretSubsystem.zeroEncoder();
        configureNamedCommands();
        configureAutoChooser();
        configureButtonBindings();
    }

    // Creates the method configureAutoChooser, which sets up the autonomous command
    // chooser with various autonomous routines and system identification routines
    // for the robot.
    @Override
    protected void configureAutoChooser() {
        this.autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
        this.autoChooser.addOption(
                "Drive Wheel Radius Characterization",
                TuningCommandsDrive.wheelRadiusCharacterization(this.swerveDriveSubsystem));
        this.autoChooser.addOption(
                "Drive Simple FF Characterization",
                this.swerveDriveSubsystem.feedforwardCharacterization());

        // add tuning mode autos
        if (RobotConstants.TUNING_MODE) {
            // Set up SysId routines
            this.autoChooser.addOption(
                    "Drive SysId (Quasistatic Forward)",
                    this.swerveDriveSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
            this.autoChooser.addOption(
                    "Drive SysId (Quasistatic Reverse)",
                    this.swerveDriveSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
            this.autoChooser.addOption(
                    "Drive SysId (Dynamic Forward)",
                    this.swerveDriveSubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward));
            this.autoChooser.addOption(
                    "Drive SysId (Dynamic Reverse)",
                    this.swerveDriveSubsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse));

            this.autoChooser.addOption(
                    "Climb Simple FF Characterization",
                    climbSubsystem.feedforwardCharacterization());
            this.autoChooser.addOption(
                    "Climb SysId (Quasistatic Forward)",
                    climbSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward));

            this.autoChooser.addOption(
                    "Feeder Simple FF Characterization",
                    feederSubsystem.feedforwardCharacterization());
            this.autoChooser.addOption(
                    "Feeder SysId (Quasistatic Forward)",
                    feederSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward));

            this.autoChooser.addOption(
                    "Hopper Simple FF Characterization",
                    hopperSubsystem.feedforwardCharacterization());
            this.autoChooser.addOption(
                    "Hopper SysId (Quasistatic Forward)",
                    hopperSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward));

            this.autoChooser.addOption(
                    "Intake Roller Simple FF Characterization",
                    intakeSubsystem.feedforwardCharacterization());
            this.autoChooser.addOption(
                    "Intake Roller SysId (Quasistatic Forward)",
                    intakeSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward));

            this.autoChooser.addOption(
                    "Shooter Simple FF Characterization",
                    shooterSubsystem.feedforwardCharacterization());
            this.autoChooser.addOption(
                    "Shooter SysId (Quasistatic Forward)",
                    shooterSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        }
    }

    @Override
    protected void configureNamedCommands() {
        // Creates the commands for using non-drive subsystems in autonomous
        new EventTrigger("ExtendIntake").onTrue(intakeSubsystem.ExtendArmCommand());
        new EventTrigger("Intake").whileTrue(intakeSubsystem.IntakeCommand());
        new EventTrigger("RetractIntake").onTrue(intakeSubsystem.RetractArmCommand());
        new EventTrigger("AimTurret").whileTrue(Commands.runOnce(() -> turretSubsystem.setAngle(0), turretSubsystem));

        NamedCommands.registerCommand("RunEverything",
                new ShootCommand(swerveDriveSubsystem, hopperSubsystem, feederSubsystem, shooterSubsystem,
                        intakeSubsystem.isIntaking(), 5.0));
    }

    @Override
    public Command getAutonomousCommand() {
        return this.autoChooser.get();
    }

    @Override
    public void teleopInit() {
        super.teleopInit();
    }

    @Override
    public void updateVisualizer() {
        // calulate pose of the turret
        Pose2d robotPose = swerveDriveSubsystem.getPose();
        Transform3d transform3d = new Transform3d(
                -FuelCellConstants.DIAMETER * 1.1,
                FuelCellConstants.DIAMETER * 1.1,
                FuelCellConstants.DIAMETER * 2.7,
                new Rotation3d(
                        0.0,
                        Units.degreesToRadians(-180.0 + TurretConstants.verticalLaunchAngle), // launch angle
                        robotPose.getRotation().getRadians() + Math.toRadians(turretSubsystem.getPosition())));
        Pose3d turretPose = new Pose3d(robotPose).transformBy(transform3d);

        mechanismVisualizer.update(
                intakeSubsystem.getPosition(),
                hopperSubsystem.getPosition(),
                turretSubsystem.getPosition(),
                climbSubsystem.getLinearDistanceInches(),
                feederSubsystem.getPosition(),
                turretPose,
                shooterSubsystem.getTangentialVelocity(),
                swerveDriveSubsystem.getModuleAngles());

        Logger.recordOutput("Current Draw/Climb", climbSubsystem.getCurrentDrawAmps());
        Logger.recordOutput("Current Draw/Feeder", feederSubsystem.getCurrentDrawAmps());
        Logger.recordOutput("Current Draw/Hopper", hopperSubsystem.getCurrentDrawAmps());
        Logger.recordOutput("Current Draw/Intake", intakeSubsystem.getCurrentDrawAmps());
        Logger.recordOutput("Current Draw/Shooter", shooterSubsystem.getCurrentDrawAmps());
        Logger.recordOutput("Current Draw/Swerve", swerveDriveSubsystem.getCurrentDrawAmps());
        Logger.recordOutput("Current Draw/Turret", turretSubsystem.getCurrentDrawAmps());
    }

    @Override
    public void updateSimulation() {
    }

    @Override
    public void onAllianceChanged(Alliance currentAlliance, int currentLocation) {
    }
}
