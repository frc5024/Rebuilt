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
import frc.robot.Constants.TurretConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.ShootForFiveSecondsCommand;
import frc.robot.commands.SpinToHubCommand;
import frc.robot.commands.TuningCommandsDrive;
import frc.robot.commands.TuningCommandsIntake;
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
                swerveDriveSubsystem,
                new VisionIOLimelight(VisionConstants.frontCamera, swerveDriveSubsystem::getRotation),
                new VisionIOLimelight(VisionConstants.rearCamera, swerveDriveSubsystem::getRotation));

        this.m_climb = new ClimbSubsystem(new ClimbModuleIOTalonFX());
        this.m_feeder = new FeederSubsystem(new FeederModuleIOSparkFlexRelativeEncoder());
        this.m_hopper = new HopperSubsystem(new HopperModuleIOSparkMaxRelativeEncoder());
        this.m_intake = new IntakeSubsystem(new ArmModuleIOSparkMaxRelativeEncoder(),
                new RollerModuleIOSparkFlexClosedLoopController());
        this.m_shooter = new ShooterSubsystem(new ShooterModuleIOSparkFlexClosedLoopController());
        this.m_turret = new TurretSubsystem(new TurretModuleIOSparkMaxRelativeEncoder());

        if (!RobotConstants.TUNING_MODE) {
            m_turret.setDefaultCommand(new SpinToHubCommand(m_turret, () -> swerveDriveSubsystem.getPose(),
                    () -> swerveDriveSubsystem.getChassisSpeeds(), () -> m_shooter.getTangentialVelocity(),
                    m_shooter::addDistanceMeasurement));
        }

        m_turret.zeroEncoder();
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

        // add tuning mode autos
        if (RobotConstants.TUNING_MODE) {
            // Set up SysId routines
            this.autoChooser.addOption(
                    "Drive Wheel Radius Characterization",
                    TuningCommandsDrive.wheelRadiusCharacterization(this.swerveDriveSubsystem));
            this.autoChooser.addOption(
                    "Drive Simple FF Characterization",
                    TuningCommandsDrive.feedforwardCharacterization(this.swerveDriveSubsystem));
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
                    "Intake Roller Simple FF Characterization",
                    TuningCommandsIntake.feedforwardCharacterization(m_intake));
        }
    }

    @Override
    protected void configureNamedCommands() {
        // Creates the commands for using non-drive subsystems in autonomous
        new EventTrigger("ExtendIntake").onTrue(m_intake.ExtendArmCommand());
        new EventTrigger("Intake").whileTrue(m_intake.IntakeCommand());
        new EventTrigger("RetractIntake").onTrue(m_intake.RetractArmCommand());
        new EventTrigger("AimTurret").whileTrue(Commands.runOnce(() -> m_turret.setAngle(0), m_turret));
        new EventTrigger("Climb").whileTrue(m_climb.contractclimb());
        new EventTrigger("Declimb").whileTrue(m_climb.extendclimb());

        NamedCommands.registerCommand("RunEverything",
                new ShootForFiveSecondsCommand(m_shooter, m_hopper, m_feeder, m_intake,
                        () -> swerveDriveSubsystem.getPose()));
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
                        robotPose.getRotation().getRadians() + Math.toRadians(m_turret.getPosition())));
        Pose3d turretPose = new Pose3d(robotPose).transformBy(transform3d);

        mechanismVisualizer.update(
                m_intake.getPosition(),
                m_hopper.getPosition(),
                m_turret.getPosition(),
                m_climb.getPosition(),
                m_feeder.getPosition(),
                turretPose,
                m_shooter.getTangentialVelocity(),
                swerveDriveSubsystem.getModuleAngles());

        Logger.recordOutput("Current Draw/Climb", m_climb.getCurrentDrawAmps());
        Logger.recordOutput("Current Draw/Feeder", m_feeder.getCurrentDrawAmps());
        Logger.recordOutput("Current Draw/Hopper", m_hopper.getCurrentDrawAmps());
        Logger.recordOutput("Current Draw/Intake", m_intake.getCurrentDrawAmps());
        Logger.recordOutput("Current Draw/Shooter", m_shooter.getCurrentDrawAmps());
        Logger.recordOutput("Current Draw/Swerve", swerveDriveSubsystem.getCurrentDrawAmps());
        Logger.recordOutput("Current Draw/Turret", m_turret.getCurrentDrawAmps());

        Logger.recordOutput("Turret/Pose", turretPose);
    }

    @Override
    public void updateSimulation() {
    }

    @Override
    public void onAllianceChanged(Alliance currentAlliance, int currentLocation) {
    }
}
