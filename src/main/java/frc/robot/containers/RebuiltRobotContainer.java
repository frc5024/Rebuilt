package frc.robot.containers;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.TuningCommands;
import frc.robot.commands.distanceShooterCommand;
import frc.robot.commands.runEverything;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.climb.ClimbModuleIOTalonFX;
import frc.robot.subsystems.climb.ClimbSubsystem;
import frc.robot.subsystems.feeder.FeederModuleIOSparkMax;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.hopper.HopperModuleIOSparkMax;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.intake.IntakeModuleIOSparkMax;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterModuleIOSparkFlex;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.GyroIOPigeon2;
import frc.robot.subsystems.swervedrive.SwerveDriveSubsystem;
import frc.robot.subsystems.swervedrive.SwerveModuleIOTalonFX;
import frc.robot.subsystems.turret.TurretModuleIOSparkMax2;
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

        // Real robot, instantiate hardware IO implementations
        this.swerveDriveSubsystem = new SwerveDriveSubsystem(
                new GyroIOPigeon2(),
                new SwerveModuleIOTalonFX(TunerConstants.FrontLeft),
                new SwerveModuleIOTalonFX(TunerConstants.FrontRight),
                new SwerveModuleIOTalonFX(TunerConstants.BackLeft),
                new SwerveModuleIOTalonFX(TunerConstants.BackRight),
                (robotPose) -> {
                });

        this.visionSubsystem = new VisionSubsystem(
                swerveDriveSubsystem::addVisionMeasurement,
                new VisionIOLimelight(VisionConstants.frontCamera, swerveDriveSubsystem::getRotation),
                new VisionIOLimelight(VisionConstants.rearCamera, swerveDriveSubsystem::getRotation));

        this.m_climb = new ClimbSubsystem(new ClimbModuleIOTalonFX());
        this.m_feeder = new FeederSubsystem(new FeederModuleIOSparkMax());
        this.m_hopper = new HopperSubsystem(new HopperModuleIOSparkMax());
        this.m_intake = new IntakeSubsystem(new IntakeModuleIOSparkMax());
        this.m_shooter = new ShooterSubsystem(new ShooterModuleIOSparkFlex());
        this.m_turret = new TurretSubsystem(new TurretModuleIOSparkMax2());

        // Creates the commands for using non-drive subsystems in autonomous

        NamedCommands.registerCommand("Shooter", m_shooter.shooterCommand());
        NamedCommands.registerCommand("DistanceShooter", new distanceShooterCommand(m_shooter, swerveDriveSubsystem));
        NamedCommands.registerCommand("Feeder", m_feeder.feederCommand());
        // NamedCommands.registerCommand("SetTurretToHub", m_turret.setAngle(0));
        NamedCommands.registerCommand("Intake", m_intake.IntakeCommand());
        NamedCommands.registerCommand("ExtendIntake", m_intake.ExtendArmCommand());
        NamedCommands.registerCommand("RetractIntake", m_intake.RetractArmCommand());
        NamedCommands.registerCommand("Outtake", m_intake.OuttakeCommand());
        NamedCommands.registerCommand("Climb", m_climb.contractclimb());
        NamedCommands.registerCommand("Declimb", m_climb.extendclimb());
        NamedCommands.registerCommand("Dontdeclimb", m_climb.dontdeclimb());
        NamedCommands.registerCommand("ExtendClimb", m_climb.extendclimb());
        NamedCommands.registerCommand("ContractClimb", m_climb.contractclimb());
        NamedCommands.registerCommand("SpinHopper", m_hopper.SpinCommand());
        NamedCommands.registerCommand("RunEverything",
                Commands.parallel(new distanceShooterCommand(m_shooter, swerveDriveSubsystem),
                        new runEverything(m_feeder, m_shooter, m_hopper),
                        Commands.waitSeconds(2).andThen(m_intake.RetractArmCommand())));

        configureAutoChooser();
        configureButtonBindings();

    }

    // Creates the method configureAutoChooser, which sets up the autonomous command
    // chooser with various autonomous routines and system identification routines
    // for the robot.
    @Override
    protected void configureAutoChooser() {
        this.autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

        // Set up SysId routines
        this.autoChooser.addOption(
                "Drive Wheel Radius Characterization",
                TuningCommands.wheelRadiusCharacterization(this.swerveDriveSubsystem));
        this.autoChooser.addOption(
                "Drive Simple FF Characterization",
                TuningCommands.feedforwardCharacterization(this.swerveDriveSubsystem));
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
    }

    @Override
    public Command getAutonomousCommand() {
        return this.autoChooser.get();
    }

    @Override
    public void updateSimulation() {
    }

    @Override
    public void onAllianceChanged(Alliance currentAlliance, int currentLocation) {
    }
}
