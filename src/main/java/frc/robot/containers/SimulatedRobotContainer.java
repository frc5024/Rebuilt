package frc.robot.containers;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.swervedrive.GyroIO;
import frc.robot.subsystems.swervedrive.SwerveModuleIOSim;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.swervedrive.SwerveDriveSubsystem;

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
                new VisionIOPhotonVisionSim(VisionConstants.camera0Name, VisionConstants.robotToCamera0,
                        this.swerveDriveSubsystem::getPose),
                new VisionIOPhotonVisionSim(VisionConstants.camera1Name, VisionConstants.robotToCamera1,
                        this.swerveDriveSubsystem::getPose));

        configureAutoChooser();
        configureButtonBindings();
    }

    @Override
    protected void configureAutoChooser() {
        this.autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
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
