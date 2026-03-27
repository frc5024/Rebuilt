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
import frc.robot.Constants.HopperConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.SpinToHubCommand;
import frc.robot.commands.TuningCommandsDrive;
import frc.robot.commands.TuningCommandsIntake;
import frc.robot.generated.TunerConstants;
import frc.robot.mechanisms.MechanismVisualizer;
import frc.robot.simulation.ShooterSubsystemSim;
import frc.robot.subsystems.climb.ClimbModuleIOSim;
import frc.robot.subsystems.climb.ClimbSubsystem;
import frc.robot.subsystems.feeder.FeederModuleIOSim;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.hopper.HopperModuleIOSim;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.intake.ArmModuleIOSim;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.RollerModuleIOSim;
import frc.robot.subsystems.shooter.ShooterModuleIOSim;
import frc.robot.subsystems.swervedrive.GyroModuleIO;
import frc.robot.subsystems.swervedrive.SwerveDriveSubsystem;
import frc.robot.subsystems.swervedrive.SwerveModuleIOSim;
import frc.robot.subsystems.turret.TurretModuleIOSim;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.util.FuelSim;
import frc.robot.util.FuelSimCount;

/**
 * 
 */
public class SimulatedRobotContainer extends RobotContainer {
    /**
     * 
     */
    public SimulatedRobotContainer() {
        super();

        // used to visually simulate intake, hopper, turret, climb
        this.mechanismVisualizer = new MechanismVisualizer();

        // used to visually simulate fuel on the field
        this.fuelSim = new FuelSim("FuelSim");
        this.fuelSimCount = new FuelSimCount(8);

        // simulated subsystems
        this.swerveDriveSubsystem = new SwerveDriveSubsystem(
                // new GyroModuleIOSim(),
                new GyroModuleIO() {
                },
                new SwerveModuleIOSim(TunerConstants.FrontLeft),
                new SwerveModuleIOSim(TunerConstants.FrontRight),
                new SwerveModuleIOSim(TunerConstants.BackLeft),
                new SwerveModuleIOSim(TunerConstants.BackRight),
                (robotPose) -> {
                });

        this.visionSubsystem = new VisionSubsystem(
                this.swerveDriveSubsystem::addVisionMeasurement,
                this.swerveDriveSubsystem,
                new VisionIOPhotonVisionSim(VisionConstants.frontCamera, this.swerveDriveSubsystem::getPose),
                new VisionIOPhotonVisionSim(VisionConstants.rearCamera, this.swerveDriveSubsystem::getPose));

        this.m_feeder = new FeederSubsystem(new FeederModuleIOSim());
        this.m_climb = new ClimbSubsystem(new ClimbModuleIOSim());
        this.m_hopper = new HopperSubsystem(new HopperModuleIOSim());
        this.m_intake = new IntakeSubsystem(new ArmModuleIOSim(), new RollerModuleIOSim());
        this.m_turret = new TurretSubsystem(new TurretModuleIOSim());
        this.m_shooter = new ShooterSubsystemSim(new ShooterModuleIOSim(), () -> m_turret.getPosition(),
                () -> m_feeder.isRunning(), fuelSim,
                fuelSimCount);

        if (!RobotConstants.TUNING_MODE) {
            this.m_turret.setDefaultCommand(new SpinToHubCommand(m_turret, () -> swerveDriveSubsystem.getPose()));
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
                        .setFuelStored(Math.min(fuelSimCount.getFuelStored() + 1, HopperConstants.CAPACITY)));

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
        new EventTrigger("Intake").whileTrue(m_intake.IntakeCommand());

        NamedCommands.registerCommand("RunEverything",
                new ShootCommand(m_shooter, m_hopper, m_feeder, m_intake, () -> swerveDriveSubsystem.getPose()));
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
                m_turret.getPosition(),
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
