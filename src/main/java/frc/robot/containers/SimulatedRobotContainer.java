package frc.robot.containers;

import java.util.List;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.TuningCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.mechanisms.MechanismVisualizer;
import frc.robot.subsystems.climb.ClimbModuleIOSim;
import frc.robot.subsystems.climb.ClimbSubsystem;
import frc.robot.subsystems.feeder.FeederModuleIOSim;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.hopper.HopperModuleIOSim;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.intake.IntakeModuleIOSim;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterModuleIOSim;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.GyroIO;
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
        // used to visually simulate intake, hopper, turret, climb
        this.mechanismVisualizer = new MechanismVisualizer();

        // used to visually simulate fuel on the field
        this.fuelSim = new FuelSim("FuelSim");
        this.fuelSimCount = new FuelSimCount(8);

        // simulated subsystems
        this.swerveDriveSubsystem = new SwerveDriveSubsystem(
                new GyroIO() {
                },
                new SwerveModuleIOSim(TunerConstants.FrontLeft),
                new SwerveModuleIOSim(TunerConstants.FrontRight),
                new SwerveModuleIOSim(TunerConstants.BackLeft),
                new SwerveModuleIOSim(TunerConstants.BackRight),
                (robotPose) -> {
                });

        this.visionSubsystem = new VisionSubsystem(
                this.swerveDriveSubsystem::addVisionMeasurement,
                new VisionIOPhotonVisionSim(VisionConstants.frontCamera, this.swerveDriveSubsystem::getPose),
                new VisionIOPhotonVisionSim(VisionConstants.rearCamera, this.swerveDriveSubsystem::getPose));

        this.m_feeder = new FeederSubsystem(new FeederModuleIOSim());
        this.m_climb = new ClimbSubsystem(new ClimbModuleIOSim());
        this.m_hopper = new HopperSubsystem(new HopperModuleIOSim());
        this.m_intake = new IntakeSubsystem(new IntakeModuleIOSim());
        this.m_shooter = new ShooterSubsystem(new ShooterModuleIOSim());
        this.m_turret = new TurretSubsystem(new TurretModuleIOSim());

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

    /**
     * 
     */
    private void configureFuelSim() {
        fuelSim.registerRobot(RobotConstants.fullWidth, RobotConstants.fullLength, Units.inchesToMeters(6),
                () -> swerveDriveSubsystem.getPose(), () -> swerveDriveSubsystem.getChassisSpeeds());

        fuelSim.registerIntake(RobotConstants.fullWidth / 2.0,
                RobotConstants.fullWidth / 2.0 + Units.inchesToMeters(8.0),
                -RobotConstants.fullLength / 2.0, RobotConstants.fullLength / 2.0,
                () -> m_intake.isIntakeExtended(),
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
    public void onAllianceChanged(Alliance alliance, int location) {
        int index = alliance == Alliance.Blue ? 0 : 1;
        location -= 1;

        Pose2d pose2d = FieldConstants.SIMULATION_START_POSES[index][location];
        swerveDriveSubsystem.setPose(pose2d);

        fuelSim.clearFuel();
        fuelSim.spawnStartingFuel();
    }

    @Override
    public void updateSimulation() {
        fuelSim.updateSim();

        mechanismVisualizer.update(
                m_intake.getPosition(),
                m_hopper.getPosition(),
                Math.sin(Timer.getTimestamp()) - 1.0,
                m_climb.getPosition(),
                new Pose3d(),
                0.0);

        RoboRioSim.setVInVoltage(
                BatterySim.calculateDefaultBatteryLoadedVoltage(
                        m_climb.getCurrentDrawAmps(),
                        m_feeder.getCurrentDrawAmps(),
                        m_hopper.getCurrentDrawAmps(),
                        m_intake.getCurrentDrawAmps(),
                        m_shooter.getCurrentDrawAmps(),
                        swerveDriveSubsystem.getCurrentDrawAmps(),
                        m_turret.getCurrentDrawAmps()));

        Logger.recordOutput("FuelSim/FuelInRobot", fuelSimCount.getFuelInRobotPoses(swerveDriveSubsystem.getPose()));
    }
}
