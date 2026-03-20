package frc.robot.containers;

import java.util.List;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.commands.LockTurretOnTarget;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.TuningCommandsDrive;
import frc.robot.commands.TuningCommandsIntake;
import frc.robot.mechanisms.MechanismVisualizer;
import frc.robot.simulation.ShooterSubsystemSim;
import frc.robot.subsystems.climb.ClimbModuleIO;
import frc.robot.subsystems.climb.ClimbSubsystem;
import frc.robot.subsystems.feeder.FeederModuleIO;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.hopper.HopperModuleIO;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.intake.ArmModuleIO;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.RollerModuleIO;
import frc.robot.subsystems.shooter.ShooterModuleIO;
import frc.robot.subsystems.swervedrive.GyroModuleIO;
import frc.robot.subsystems.swervedrive.SwerveDriveSubsystem;
import frc.robot.subsystems.swervedrive.SwerveModuleIO;
import frc.robot.subsystems.turret.TurretModuleIO;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.util.FuelSim;
import frc.robot.util.FuelSimCount;

/**
 * 
 */
public class ReplayingRobotContainer extends RobotContainer {
    /**
     * 
     */
    public ReplayingRobotContainer() {
        super();

        // used to visually simulate intake, hopper, turret, climb
        this.mechanismVisualizer = new MechanismVisualizer();

        // used to visually simulate fuel on the field
        this.fuelSim = new FuelSim("FuelSim");
        this.fuelSimCount = new FuelSimCount(8);

        // simulated subsystems
        this.swerveDriveSubsystem = new SwerveDriveSubsystem(
                new GyroModuleIO() {
                },
                new SwerveModuleIO() {
                },
                new SwerveModuleIO() {
                },
                new SwerveModuleIO() {
                },
                new SwerveModuleIO() {
                },
                (robotPose) -> {
                });

        this.visionSubsystem = new VisionSubsystem(
                this.swerveDriveSubsystem::addVisionMeasurement,
                this.swerveDriveSubsystem,
                new VisionIO() {
                },
                new VisionIO() {
                });

        this.m_feeder = new FeederSubsystem(new FeederModuleIO() {
        });
        this.m_climb = new ClimbSubsystem(new ClimbModuleIO() {
        });
        this.m_hopper = new HopperSubsystem(new HopperModuleIO() {
        });
        this.m_intake = new IntakeSubsystem(new ArmModuleIO() {
        }, new RollerModuleIO() {
        });
        this.m_turret = new TurretSubsystem(new TurretModuleIO() {
        });
        this.m_shooter = new ShooterSubsystemSim(new ShooterModuleIO() {
        }, () -> m_turret.getCurrentAngle(),
                () -> m_feeder.isRunning(), fuelSim,
                fuelSimCount);

        if (!RobotConstants.TUNING_MODE) {
            this.m_turret.setDefaultCommand(new LockTurretOnTarget(m_turret, () -> swerveDriveSubsystem.getPose(),
                    () -> swerveDriveSubsystem.getChassisSpeeds(), () -> m_shooter.getTangentialVelocity()));
            // this.m_turret.setDefaultCommand(new spinToHubCommand(m_turret, () ->
            // swerveDriveSubsystem.getPose(),
            // () -> swerveDriveSubsystem.getChassisSpeeds()));
        }

        configureNamedCommands();
        configureAutoChooser();
        configureButtonBindings();
        configureFuelSim();

        // AK any paths driven by the autobuilder
        PathPlannerLogging.setLogActivePathCallback((List<Pose2d> poses) -> {
            Logger.recordOutput("PathPlanner/ActivePath", poses.toArray(new Pose2d[0]));
        });
    }

    @Override
    protected void configureAutoChooser() {
        this.autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

        if (RobotConstants.TUNING_MODE) {
            // Set up SysId routines
            this.autoChooser.addOption(
                    "Drive Wheel Radius Characterization",
                    TuningCommandsDrive.wheelRadiusCharacterization(swerveDriveSubsystem));
            this.autoChooser.addOption(
                    "Drive Simple FF Characterization",
                    TuningCommandsDrive.feedforwardCharacterization(swerveDriveSubsystem));
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

    /**
     * 
     */
    private void configureFuelSim() {
        fuelSim.registerRobot(RobotConstants.fullWidth, RobotConstants.fullLength, Units.inchesToMeters(6),
                () -> swerveDriveSubsystem.getPose(), () -> swerveDriveSubsystem.getChassisSpeeds());

        fuelSim.registerIntake(RobotConstants.fullWidth / 2.0,
                RobotConstants.fullWidth / 2.0 + Units.inchesToMeters(8.0),
                -RobotConstants.fullLength / 2.0, RobotConstants.fullLength / 2.0,
                () -> m_intake.isExtended(),
                () -> fuelSimCount
                        .setFuelStored(Math.min(fuelSimCount.getFuelStored() + 1, fuelSimCount.getCapacity())));

        fuelSim.setSubticks(1);
        fuelSim.enableAirResistance();

        fuelSim.start();
        fuelSim.spawnStartingFuel();

        RobotModeTriggers.autonomous()
                .onTrue(
                        Commands.runOnce(
                                () -> {
                                    fuelSim.clearFuel();
                                    fuelSim.spawnStartingFuel();
                                    fuelSimCount.setFuelStored(8);
                                }));
    }

    @Override
    public void configureNamedCommands() {
        new EventTrigger("ExtendIntake").onTrue(m_intake.ExtendArmCommand());

        /**
         * new EventTrigger("Intake").whileTrue(m_intake.IntakeCommand());
         * new EventTrigger("RetractIntake").onTrue(m_intake.RetractArmCommand());
         * new EventTrigger("AimTurret").whileTrue(
         * Commands.runOnce(() -> m_turret.setAngle(0), m_turret));
         * new EventTrigger("Climb").whileTrue(m_climb.contractclimb());
         * new EventTrigger("Declimb").whileTrue(m_climb.extendclimb());
         * 
         * // NamedCommands.registerCommand("Shooter", m_shooter.shooterCommand());
         * // NamedCommands.registerCommand("DistanceShooter", new
         * // distanceShooterCommand(m_shooter, swerveDriveSubsystem));
         * // NamedCommands.registerCommand("Feeder", m_feeder.feederCommand());
         * // NamedCommands.registerCommand("Intake", m_intake.IntakeCommand());
         * // NamedCommands.registerCommand("ExtendIntake",
         * m_intake.ExtendArmCommand());
         * // NamedCommands.registerCommand("RetractIntake",
         * m_intake.RetractArmCommand());
         * // NamedCommands.registerCommand("Outtake", m_intake.OuttakeCommand());
         * // NamedCommands.registerCommand("Climb", m_climb.contractclimb());
         * // NamedCommands.registerCommand("Declimb", m_climb.extendclimb());
         * // NamedCommands.registerCommand("Dontdeclimb", m_climb.dontdeclimb());
         * // NamedCommands.registerCommand("ExtendClimb", m_climb.extendclimb());
         * // NamedCommands.registerCommand("ContractClimb", m_climb.contractclimb());
         * // NamedCommands.registerCommand("SpinHopper", m_hopper.SpinCommand());
         */
        NamedCommands.registerCommand("RunEverything",
                Commands.parallel(
                        new ShootCommand(m_shooter, m_hopper, m_feeder, () -> swerveDriveSubsystem.getPose()),
                        Commands.waitSeconds(2).andThen(Commands.runOnce(() -> m_intake.retractArm()))));

        NamedCommands.registerCommand("RunEverythings",
                Commands.parallel(
                        new ShootCommand(m_shooter, m_hopper, m_feeder, () -> swerveDriveSubsystem.getPose()),
                        // new distanceShooterCommand(m_shooter, swerveDriveSubsystem),
                        // new runEverything(m_feeder, m_shooter, m_hopper),
                        Commands.waitSeconds(2).andThen(m_intake.RetractArmCommand())));
    }

    @Override
    public void onAllianceChanged(Alliance alliance, int location) {
        int index = alliance == Alliance.Blue ? 0 : 1;
        location -= 1;

        Pose2d pose2d = FieldConstants.SIMULATION_START_POSES[index][location];
        swerveDriveSubsystem.setPose(pose2d);

        fuelSim.clearFuel();
        fuelSim.spawnStartingFuel();
    }

    @Override
    public void updateVisualizer() {
        // calulate pose of the turret
        Pose2d robotPose = swerveDriveSubsystem.getPose();
        Pose3d turretPose = m_turret.getPose(robotPose);

        mechanismVisualizer.update(
                m_intake.getPosition(),
                m_hopper.getPosition(),
                m_turret.getCurrentAngle(),
                m_climb.getPosition(),
                m_feeder.getPosition(),
                turretPose,
                m_shooter.getTangentialVelocity(),
                swerveDriveSubsystem.getModuleAngles());

        Logger.recordOutput("Turret/Pose", turretPose);
    }

    @Override
    public void updateSimulation() {
        fuelSim.updateSim();

        RoboRioSim.setVInVoltage(
                BatterySim.calculateDefaultBatteryLoadedVoltage(
                        m_climb.getCurrentDrawAmps(),
                        m_feeder.getCurrentDrawAmps(),
                        m_hopper.getCurrentDrawAmps(),
                        m_intake.getCurrentDrawAmps(),
                        m_shooter.getCurrentDrawAmps(),
                        swerveDriveSubsystem.getCurrentDrawAmps(),
                        m_turret.getCurrentDrawAmps()));

        Logger.recordOutput("Current Draw/Climb", m_climb.getCurrentDrawAmps());
        Logger.recordOutput("Current Draw/Feeder", m_feeder.getCurrentDrawAmps());
        Logger.recordOutput("Current Draw/Hopper", m_hopper.getCurrentDrawAmps());
        Logger.recordOutput("Current Draw/Intake", m_intake.getCurrentDrawAmps());
        Logger.recordOutput("Current Draw/Shooter", m_shooter.getCurrentDrawAmps());
        Logger.recordOutput("Current Draw/Swerve", swerveDriveSubsystem.getCurrentDrawAmps());
        Logger.recordOutput("Current Draw/Turret", m_turret.getCurrentDrawAmps());

        Logger.recordOutput("FuelSim/FuelInRobotCount", fuelSimCount.getFuelStored());
        Logger.recordOutput("FuelSim/FuelInRobot", fuelSimCount.getFuelInRobotPoses(swerveDriveSubsystem.getPose()));
        Logger.recordOutput("FuelSim/BlueHubScore", FuelSim.Hub.BLUE_HUB.getScore());
        Logger.recordOutput("FuelSim/RedHubScore", FuelSim.Hub.RED_HUB.getScore());
    }
}
