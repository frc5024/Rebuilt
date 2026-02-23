package frc.robot.containers;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.TuningCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.feeder.FeederModuleIOSim;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.swervedrive.GyroIO;
import frc.robot.subsystems.swervedrive.SwerveDriveSubsystem;
import frc.robot.subsystems.swervedrive.SwerveModuleIOSim;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.subsystems.vision.VisionSubsystem;

/**
 * 
 */
public class SimulatedRobotContainer extends RobotContainer {
    /**
     * 
     */
    public SimulatedRobotContainer() {
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

        configureAutoChooser();
        configureButtonBindings();
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

    @Override
    public void onAllianceChanged(Alliance alliance, int location) {
        int index = alliance == Alliance.Blue ? 0 : 1;
        location -= 1;

        Pose2d pose2d = FieldConstants.STATION_POSES[index][location];
        this.swerveDriveSubsystem.setPose(pose2d);
        resetSimulationField(pose2d);
    }

    /**
     * 
     */
    public void resetSimulationField(Pose2d pose2d) {
    }

    @Override
    public void updateSimulation() {
    }
}
