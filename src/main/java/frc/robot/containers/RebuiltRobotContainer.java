package frc.robot.containers;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.shooterCommand;
import frc.robot.commands.Hopper.Spin;
import frc.robot.Constants.VisionConstants;
import frc.robot.Robot;
import frc.robot.commands.TuningCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.swervedrive.GyroIOPigeon2;
import frc.robot.subsystems.swervedrive.SwerveModuleIOTalonFX;
import frc.robot.subsystems.swervedrive.SwerveDriveSubsystem;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.shooter;
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
                                new SwerveModuleIOTalonFX(TunerConstants.FrontLeft),
                                new SwerveModuleIOTalonFX(TunerConstants.FrontRight),
                                new SwerveModuleIOTalonFX(TunerConstants.BackLeft),
                                new SwerveModuleIOTalonFX(TunerConstants.BackRight),
                                (robotPose) -> {
                                });
                this.visionSubsystem = new VisionSubsystem(
                                swerveDriveSubsystem::addVisionMeasurement,
                                new VisionIOLimelight(VisionConstants.camera0Name, swerveDriveSubsystem::getRotation),
                                new VisionIOLimelight(VisionConstants.camera1Name, swerveDriveSubsystem::getRotation));

                //Creates the commands for using non-drive subsystems in autonomous

                NamedCommands.registerCommand("Shooter", m_shooter.shooterCommand());
                NamedCommands.registerCommand("Feeder", m_feeder.feederCommand());
                NamedCommands.registerCommand("Intake", m_intake.IntakeSpin());
                NamedCommands.registerCommand("ExtendIntake", m_intake.ExtendSpin());
                NamedCommands.registerCommand("RetractIntake", m_intake.RetractSpin());
                NamedCommands.registerCommand("Outtake", m_intake.OuttakeSpin());
                NamedCommands.registerCommand("Climb", m_climb.climb());
                NamedCommands.registerCommand("Declimb", m_climb.declimb());
                NamedCommands.registerCommand("Dontdeclimb", m_climb.dontdeclimb());
                NamedCommands.registerCommand("SpinHopper", m_hopper.SpinCommand());


                configureAutoChooser();
                configureButtonBindings();

                

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
