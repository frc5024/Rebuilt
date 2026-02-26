package frc.robot.containers;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.FuelCellConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.mechanisms.MechanismVisualizer;
import frc.robot.simulation.IntakeSubsystemSim;
import frc.robot.simulation.ShooterSubsystemSim;
import frc.robot.subsystems.climb.ClimbModuleIOSim;
import frc.robot.subsystems.climb.ClimbSubsystem;
import frc.robot.subsystems.feeder.FeederModuleIOSim;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.hopper.HopperModuleIOSim;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.intake.IntakeModuleIOSim;
import frc.robot.subsystems.shooter.ShooterModuleIOSim;
import frc.robot.subsystems.swervedrive.GyroIOSim;
import frc.robot.subsystems.swervedrive.SwerveDriveSubsystem;
import frc.robot.subsystems.swervedrive.SwerveModuleIOMapleSim;
import frc.robot.subsystems.turret.TurretModuleIOSim;
import frc.robot.subsystems.turret.TurretSubsystem;
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

        this.m_feeder = new FeederSubsystem(new FeederModuleIOSim());
        this.m_climb = new ClimbSubsystem(new ClimbModuleIOSim());
        this.m_hopper = new HopperSubsystem(new HopperModuleIOSim());
        this.m_intake = new IntakeSubsystemSim(new IntakeModuleIOSim());
        this.m_shooter = new ShooterSubsystemSim(new ShooterModuleIOSim());
        this.m_turret = new TurretSubsystem(new TurretModuleIOSim());

        this.mechanismVisualizer = new MechanismVisualizer();

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
    public void updateMechanisms() {

        // Pose of the turret
        Pose2d robotPose = MapleSimUtil.getSwerveDriveSimulation().getSimulatedDriveTrainPose();
        Pose3d turretPose = new Pose3d(
                robotPose.getX() - FuelCellConstants.DIAMETER,
                robotPose.getY() + (FuelCellConstants.DIAMETER * 3),
                FuelCellConstants.DIAMETER * 3,
                new Rotation3d(0.0, -Units.degreesToRadians(60.0),
                        robotPose.getRotation().getRadians() + Math.toRadians(m_turret.getTurretAngle())));

        mechanismVisualizer.update(
                m_intake.getPosition(),
                m_hopper.getPosition(),
                m_turret.getTurretAngle(),
                m_climb.getPosition(),
                turretPose,
                m_shooter.getTangentialVelocity());
    }

    @Override
    public void updateSimulation() {
        SimulatedArena.getInstance().simulationPeriodic();
        Logger.recordOutput("FieldSimulation/RobotPosition",
                MapleSimUtil.getSwerveDriveSimulation().getSimulatedDriveTrainPose());
        Logger.recordOutput("FieldSimulation/Fuel", SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel"));
    }
}
