package frc.robot.containers;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.swervedrive.GyroIOSim;
import frc.robot.subsystems.swervedrive.SwerveDriveSubsystem;
import frc.robot.subsystems.swervedrive.SwerveModuleIOMapleSim;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.util.MapleSimUtil;

/**
 * 
 */
public class MapleSimRobotContainer extends RobotContainer {
    /**
     *
     */
    public MapleSimRobotContainer() {
        super();

        // Sim robot, instantiate physics sim IO implementations
        SwerveDriveSimulation swerveDriveSimulation = MapleSimUtil.getSwerveDriveSimulation();
        SimulatedArena.getInstance().addDriveTrainSimulation(swerveDriveSimulation);

        this.swerveDriveSubsystem = new SwerveDriveSubsystem(
                new GyroIOSim(swerveDriveSimulation.getGyroSimulation()),
                new SwerveModuleIOMapleSim(swerveDriveSimulation.getModules()[0]),
                new SwerveModuleIOMapleSim(swerveDriveSimulation.getModules()[1]),
                new SwerveModuleIOMapleSim(swerveDriveSimulation.getModules()[2]),
                new SwerveModuleIOMapleSim(swerveDriveSimulation.getModules()[3]),
                swerveDriveSimulation::setSimulationWorldPose);

        this.visionSubsystem = new VisionSubsystem(
                this.swerveDriveSubsystem::addVisionMeasurement,
                new VisionIOPhotonVisionSim(VisionConstants.frontCamera, this.swerveDriveSubsystem::getPose),
                new VisionIOPhotonVisionSim(VisionConstants.rearCamera, this.swerveDriveSubsystem::getPose));

        configureAutoChooser();
        configureButtonBindings();
    }

    @Override
    protected void configureAutoChooser() {
        autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
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
        MapleSimUtil.getSwerveDriveSimulation().setSimulationWorldPose(pose2d);
        SimulatedArena.getInstance().resetFieldForAuto();
    }

    @Override
    public void updateSimulation() {
        SimulatedArena.getInstance().simulationPeriodic();
        Logger.recordOutput("FieldSimulation/RobotPosition",
                MapleSimUtil.getSwerveDriveSimulation().getSimulatedDriveTrainPose());
        // Logger.recordOutput("FieldSimulation/Coral",
        // SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
        // Logger.recordOutput("FieldSimulation/Algae",
        // SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
    }
}
