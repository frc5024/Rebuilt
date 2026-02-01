package frc.robot.containers;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.VisionConstants;
import frc.robot.Robot;
import frc.robot.commands.TuningCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.swervedrive.GyroIOPigeon2;
import frc.robot.subsystems.swervedrive.ModuleIOTalonFX;
import frc.robot.subsystems.swervedrive.SwerveDriveSubsystem;
import frc.robot.subsystems.vision.VisionIOLimelight;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * 
 */
public class RebuiltRobotContainer extends RobotContainer {

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RebuiltRobotContainer() {
                super();

                // Real robot, instantiate hardware IO implementations
                swerveDriveSubsystem = new SwerveDriveSubsystem(
                                new GyroIOPigeon2(),
                                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                                new ModuleIOTalonFX(TunerConstants.FrontRight),
                                new ModuleIOTalonFX(TunerConstants.BackLeft),
                                new ModuleIOTalonFX(TunerConstants.BackRight),
                                (robotPose) -> {
                                });
                visionSubsystem = new VisionSubsystem(
                                swerveDriveSubsystem::addVisionMeasurement,
                                new VisionIOLimelight(VisionConstants.camera0Name, swerveDriveSubsystem::getRotation),
                                new VisionIOLimelight(VisionConstants.camera1Name, swerveDriveSubsystem::getRotation));
                configureAutoChooser();
                configureButtonBindings();
        }

        // Creates the method configureAutoChooser, which sets up the autonomous command
        // chooser with various autonomous routines and system identification routines
        // for the robot.
        @Override
        protected void configureAutoChooser() {
                autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

                // Set up SysId routines
                autoChooser.addOption(
                                "Drive Wheel Radius Characterization",
                                TuningCommands.wheelRadiusCharacterization(swerveDriveSubsystem));
                autoChooser.addOption(
                                "Drive Simple FF Characterization",
                                TuningCommands.feedforwardCharacterization(swerveDriveSubsystem));
                autoChooser.addOption(
                                "Drive SysId (Quasistatic Forward)",
                                swerveDriveSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
                autoChooser.addOption(
                                "Drive SysId (Quasistatic Reverse)",
                                swerveDriveSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
                autoChooser.addOption(
                                "Drive SysId (Dynamic Forward)",
                                swerveDriveSubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward));
                autoChooser.addOption(
                                "Drive SysId (Dynamic Reverse)",
                                swerveDriveSubsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse));
        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                return autoChooser.get();
        }

        // Called periodically in simulation
        public void updateSimulation() {
        }

        public void onAllianceChanged(Alliance currentAlliance, int currentLocation) {
        }
}
