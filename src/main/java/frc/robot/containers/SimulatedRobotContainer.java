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
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.HopperConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.LockTurretOnTarget;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.TuningCommandsDrive;
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

        this.feederSubsystem = new FeederSubsystem(new FeederModuleIOSim());
        this.climbSubsystem = new ClimbSubsystem(new ClimbModuleIOSim());
        this.hopperSubsystem = new HopperSubsystem(new HopperModuleIOSim());
        this.intakeSubsystem = new IntakeSubsystem(new ArmModuleIOSim(), new RollerModuleIOSim());
        this.turretSubsystem = new TurretSubsystem(new TurretModuleIOSim());
        this.shooterSubsystem = new ShooterSubsystemSim(new ShooterModuleIOSim(), () -> turretSubsystem.getPosition(),
                () -> feederSubsystem.isRunning(), fuelSim,
                fuelSimCount);

        if (!RobotConstants.TUNING_MODE) {
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

        configureNamedCommands();
        configureAutoChooser();
        configureButtonBindings();
        configureFuelSim();

        // AK any paths driven by the autobuilder
        PathPlannerLogging.setLogActivePathCallback((List<Pose2d> poses) -> {
            Logger.recordOutput("PathPlanner/ActivePath", poses.toArray(new Pose2d[0]));
        });

        CommandScheduler.getInstance().onCommandInterrupt(
                (command, interrupter) -> {
                    System.out.println("Command " + command.getName() +
                            " was interrupted by " + (interrupter.isPresent() ? interrupter.get().getName() : "None"));
                });
    }

    @Override
    protected void configureAutoChooser() {
        this.autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
        this.autoChooser.addOption(
                "Drive Wheel Radius Characterization",
                TuningCommandsDrive.wheelRadiusCharacterization(swerveDriveSubsystem));
        this.autoChooser.addOption(
                "Drive Simple FF Characterization",
                swerveDriveSubsystem.feedforwardCharacterization());

        if (RobotConstants.TUNING_MODE) {
            // Set up SysId routines
            this.autoChooser.addOption(
                    "Drive SysId (Quasistatic Forward)",
                    swerveDriveSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
            this.autoChooser.addOption(
                    "Drive SysId (Quasistatic Reverse)",
                    swerveDriveSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
            this.autoChooser.addOption(
                    "Drive SysId (Dynamic Forward)",
                    swerveDriveSubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward));
            this.autoChooser.addOption(
                    "Drive SysId (Dynamic Reverse)",
                    swerveDriveSubsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse));

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
        new EventTrigger("ExtendIntake").onTrue(Commands.runOnce(() -> intakeSubsystem.extendArm()));
        new EventTrigger("Intake").whileTrue(Commands.runOnce(() -> intakeSubsystem.intakeRoller()));
        new EventTrigger("Retract").whileTrue(Commands.runOnce(() -> intakeSubsystem.retractArm()));

        NamedCommands.registerCommand("ShootFor5Seconds",
                new ShootCommand(swerveDriveSubsystem, hopperSubsystem, feederSubsystem, shooterSubsystem, 5.0));
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
                climbSubsystem.getLinearDistanceInches(),
                feederSubsystem.getPosition(),
                turretPose,
                shooterSubsystem.getTangentialVelocity(),
                swerveDriveSubsystem.getModuleAngles());
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
