package frc.robot.containers;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.vision.Vision;
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

        drive = new Drive(
                new GyroIOSim(swerveDriveSimulation.getGyroSimulation()),
                new ModuleIOSim(swerveDriveSimulation.getModules()[0]),
                new ModuleIOSim(swerveDriveSimulation.getModules()[1]),
                new ModuleIOSim(swerveDriveSimulation.getModules()[2]),
                new ModuleIOSim(swerveDriveSimulation.getModules()[3]),
                swerveDriveSimulation::setSimulationWorldPose);
        vision = new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(camera0Name, robotToCamera0, drive::getPose),
                new VisionIOPhotonVisionSim(camera1Name, robotToCamera1, drive::getPose));

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
        drive.setPose(pose2d);
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
