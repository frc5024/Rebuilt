package frc.robot.containers;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.VisionConstants;
import frc.robot.Robot;
import frc.robot.commands.TuningCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.blower.BlowerModuleIOTalonFX;
import frc.robot.subsystems.blower.BlowerSubsystem;
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
                this.swerveDriveSubsystem = new SwerveDriveSubsystem(
                                new GyroIOPigeon2(),
                                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                                new ModuleIOTalonFX(TunerConstants.FrontRight),
                                new ModuleIOTalonFX(TunerConstants.BackLeft),
                                new ModuleIOTalonFX(TunerConstants.BackRight),
                                (robotPose) -> {
                                });

                this.visionSubsystem = new VisionSubsystem(
                                swerveDriveSubsystem::addVisionMeasurement,
                                new VisionIOLimelight(VisionConstants.camera0Name,
                                                this.swerveDriveSubsystem::getRotation),
                                new VisionIOLimelight(VisionConstants.camera1Name,
                                                this.swerveDriveSubsystem::getRotation));

                configureNamedCommands();
                configureAutoChooser();
                configureButtonBindings();
        }

        @Override
        protected void configureNamedCommands() {
                NamedCommands.registerCommand("StartBlower", new InstantCommand(() -> {
                        this.blowerSubsystem.addAction(BlowerSubsystem.Action.START);
                }));
        }

        // Creates the method configureAutoChooser, which sets up the autonomous command
        // chooser with various autonomous routines and system identification routines
        // for the robot.
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

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                return this.autoChooser.get();
        }

        // Called periodically in simulation
        public void updateSimulation() {
        }

        public void onAllianceChanged(Alliance currentAlliance, int currentLocation) {
        }
}
