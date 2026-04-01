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
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.HopperConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.commands.ShootCommand;
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

        this.feederSubsystem = new FeederSubsystem(new FeederModuleIO() {
        });
        this.climbSubsystem = new ClimbSubsystem(new ClimbModuleIO() {
        });
        this.hopperSubsystem = new HopperSubsystem(new HopperModuleIO() {
        });
        this.intakeSubsystem = new IntakeSubsystem(new ArmModuleIO() {
        }, new RollerModuleIO() {
        });
        this.turretSubsystem = new TurretSubsystem(new TurretModuleIO() {
        });
        this.shooterSubsystem = new ShooterSubsystemSim(new ShooterModuleIO() {
        }, () -> turretSubsystem.getPosition(),
                () -> feederSubsystem.isRunning(), fuelSim,
                fuelSimCount);

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
                () -> intakeSubsystem.isExtended(),
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
        new EventTrigger("ExtendIntake").onTrue(intakeSubsystem.ExtendArmCommand());

        NamedCommands.registerCommand("RunEverything",
                new ShootCommand(swerveDriveSubsystem, shooterSubsystem, hopperSubsystem, feederSubsystem,
                        () -> swerveDriveSubsystem.getPose()));
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
        Pose3d turretPose = turretSubsystem.getPose(robotPose);

        mechanismVisualizer.update(
                intakeSubsystem.getPosition(),
                hopperSubsystem.getPosition(),
                turretSubsystem.getPosition(),
                climbSubsystem.getPosition(),
                feederSubsystem.getPosition(),
                turretPose,
                shooterSubsystem.getTangentialVelocity(),
                swerveDriveSubsystem.getModuleAngles());

        Logger.recordOutput("Turret/Pose", turretPose);
    }

    @Override
    public void updateSimulation() {
        fuelSim.updateSim();

        RoboRioSim.setVInVoltage(
                BatterySim.calculateDefaultBatteryLoadedVoltage(
                        climbSubsystem.getCurrentDrawAmps(),
                        feederSubsystem.getCurrentDrawAmps(),
                        hopperSubsystem.getCurrentDrawAmps(),
                        intakeSubsystem.getCurrentDrawAmps(),
                        shooterSubsystem.getCurrentDrawAmps(),
                        swerveDriveSubsystem.getCurrentDrawAmps(),
                        turretSubsystem.getCurrentDrawAmps()));

        Logger.recordOutput("Current Draw/Climb", climbSubsystem.getCurrentDrawAmps());
        Logger.recordOutput("Current Draw/Feeder", feederSubsystem.getCurrentDrawAmps());
        Logger.recordOutput("Current Draw/Hopper", hopperSubsystem.getCurrentDrawAmps());
        Logger.recordOutput("Current Draw/Intake", intakeSubsystem.getCurrentDrawAmps());
        Logger.recordOutput("Current Draw/Shooter", shooterSubsystem.getCurrentDrawAmps());
        Logger.recordOutput("Current Draw/Swerve", swerveDriveSubsystem.getCurrentDrawAmps());
        Logger.recordOutput("Current Draw/Turret", turretSubsystem.getCurrentDrawAmps());

        Logger.recordOutput("FuelSim/FuelInRobotCount", fuelSimCount.getFuelStored());
        Logger.recordOutput("FuelSim/FuelInRobot", fuelSimCount.getFuelInRobotPoses(swerveDriveSubsystem.getPose()));
        Logger.recordOutput("FuelSim/BlueHubScore", FuelSim.Hub.BLUE_HUB.getScore());
        Logger.recordOutput("FuelSim/RedHubScore", FuelSim.Hub.RED_HUB.getScore());
    }
}
