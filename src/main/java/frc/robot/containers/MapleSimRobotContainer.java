package frc.robot.containers;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.swervedrive.GyroIOSim;
import frc.robot.subsystems.swervedrive.ModuleIOSim;
import frc.robot.subsystems.swervedrive.SwerveDriveSubsystem;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.util.MapleSimUtil;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;

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

        swerveDriveSubsystem = new SwerveDriveSubsystem(
                new GyroIOSim(swerveDriveSimulation.getGyroSimulation()),
                new ModuleIOSim(swerveDriveSimulation.getModules()[0]),
                new ModuleIOSim(swerveDriveSimulation.getModules()[1]),
                new ModuleIOSim(swerveDriveSimulation.getModules()[2]),
                new ModuleIOSim(swerveDriveSimulation.getModules()[3]),
                swerveDriveSimulation::setSimulationWorldPose);
        visionSubsystem = new VisionSubsystem(
                swerveDriveSubsystem::addVisionMeasurement,
                new VisionIOPhotonVisionSim(VisionConstants.camera0Name, VisionConstants.robotToCamera0, swerveDriveSubsystem::getPose),
                new VisionIOPhotonVisionSim(VisionConstants.camera1Name, VisionConstants.robotToCamera1, swerveDriveSubsystem::getPose));

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

        Pose2d pose2d = Constants.STATION_POSES[index][location];
        swerveDriveSubsystem.setPose(pose2d);
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
        Logger.recordOutput(
                "FieldSimulation/RobotPosition", MapleSimUtil.getSwerveDriveSimulation().getSimulatedDriveTrainPose());
        Logger.recordOutput(
                "FieldSimulation/Coral", SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
        Logger.recordOutput(
                "FieldSimulation/Algae", SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
    }
}
