package frc.robot.commands;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.util.GameUtil;
import frc.sotm.ProjectileSimulator;
import frc.sotm.ProjectileSimulator.GeneratedLUT;
import frc.sotm.ProjectileSimulator.SimParameters;
import frc.sotm.ShotCalculator;

/**
 * 
 */
public class SOTM extends Command {
    // Subsystems
    private final TurretSubsystem turretSubsystem;
    private final Supplier<Pose2d> robotPoseSupplier;
    private final Supplier<ChassisSpeeds> chassisSpeedSupplier;

    ShotCalculator.Config config = new ShotCalculator.Config();

    ShotCalculator shotCalculator = new ShotCalculator(config);

    SimParameters params = new SimParameters(
            0.215, // ball mass kg
            0.1501, // ball diameter m
            0.47, // drag coeff (smooth sphere)
            0.2, // Magnus coeff
            1.225, // air density kg/m^3
            0.49, // exit height from floor, measure from CAD
            0.1016, // wheel diameter, measure with calipers
            1.83, // target height, from game manual
            0.6, // slip factor (0=no grip, 1=perfect), tune on robot
            60.0, // launch angle degrees from horizontal
            0.001, // sim timestep
            2800, 5100, 25, 5.0 // RPM range, search iters, max sim time
    );

    ProjectileSimulator sim = new ProjectileSimulator(params);

    GeneratedLUT lut = sim.generateLUT();

    /**
     * 
     */
    public SOTM(TurretSubsystem turretSubsystem, Supplier<Pose2d> robotPoseSupplier,
            Supplier<ChassisSpeeds> chassisSpeedSupplier) {
        this.turretSubsystem = turretSubsystem;
        this.robotPoseSupplier = robotPoseSupplier;
        this.chassisSpeedSupplier = chassisSpeedSupplier;

        for (var entry : lut.entries()) {
            if (entry.reachable()) {
                shotCalculator.loadLUTEntry(entry.distanceM(), entry.rpm(), entry.tof());
            }

        }

        addRequirements(turretSubsystem);
    }

    @Override
    public void initialize() {
        turretSubsystem.enablePID();
    }

    @Override
    public void execute() {
        Translation2d hubCenter = new Translation2d(4.6, 4.0);
        Translation2d hubForward = new Translation2d(1, 0);

        Pose2d robotPose = robotPoseSupplier.get();

        ChassisSpeeds robotSpeed = chassisSpeedSupplier.get();

        double vx_r = robotSpeed.vxMetersPerSecond;
        double vy_r = robotSpeed.vyMetersPerSecond;
        double vomega = robotSpeed.omegaRadiansPerSecond;
        Rotation2d omega = robotPose.getRotation();

        ChassisSpeeds fieldVelocity = ChassisSpeeds.fromRobotRelativeSpeeds(vx_r, vy_r, vomega, omega);

        ShotCalculator.ShotInputs inputs = new ShotCalculator.ShotInputs(
                robotPose,
                robotSpeed,
                fieldVelocity,
                hubCenter,
                hubForward,
                0.9, // vision confidence, 0 to 1
                0,
                0);

        ShotCalculator.LaunchParameters shot = shotCalculator.calculate(inputs);
        if (shot.isValid() && shot.confidence() > 50) {
            turretSubsystem.setAngle(shot.driveAngle().getDegrees());
            // shot.driveAngularVelocityRadPerSec() gives you a heading feedforward if you
            // want it
        }

    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        turretSubsystem.disablePID();

        Logger.recordOutput("Turret/Active Command", "");
    }

    /**
     * 
     */
    private Pose2d getTargetPose(Pose2d robotPose) {
        boolean isRedAlliance = DriverStation.getAlliance().get() == Alliance.Red;
        boolean isAboveMidLine = GameUtil.isAboveMidLine(robotPose);

        if (GameUtil.inAllianceZone(robotPose)) {
            return GameUtil.getHubPose();
        } else {
            return FieldConstants.MULE_POSES[isRedAlliance ? 1 : 0][isAboveMidLine ? 1 : 0];
        }
    }
}
