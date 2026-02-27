package frc.robot.containers;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Robot;
import frc.robot.controllers.ButtonBindings;
import frc.robot.controllers.ButtonsBindingsSim;
import frc.robot.mechanisms.MechanismVisualizer;
import frc.robot.subsystems.climb.ClimbSubsystem;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.SwerveDriveSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

/**
 * 
 */
abstract public class RobotContainer {
    /* Subsystems */
    protected SwerveDriveSubsystem swerveDriveSubsystem;
    protected VisionSubsystem visionSubsystem;
    protected ClimbSubsystem m_climb;
    protected FeederSubsystem m_feeder;
    protected HopperSubsystem m_hopper;
    protected IntakeSubsystem m_intake;
    protected ShooterSubsystem m_shooter;
    protected TurretSubsystem m_turret;

    /* Mechanisms */
    protected MechanismVisualizer mechanismVisualizer;

    /* Autonomous */
    protected LoggedDashboardChooser<Command> autoChooser;

    /* Controllers */
    public static CommandXboxController driverController;
    public static CommandXboxController operatorController;

    abstract protected void configureAutoChooser();

    protected void configureButtonBindings() {
        if (Robot.isReal()) {
            ButtonBindings buttonBindings = new ButtonBindings(swerveDriveSubsystem, m_climb, m_feeder, m_hopper,
                    m_intake, m_shooter, m_turret);

            driverController = buttonBindings.getDriverController();
            operatorController = buttonBindings.getOperatorController();

        } else {
            ButtonsBindingsSim buttonBindings = new ButtonsBindingsSim(swerveDriveSubsystem, m_climb, m_feeder,
                    m_hopper,
                    m_intake, m_shooter, m_turret);

            driverController = buttonBindings.getDriverController();
            operatorController = buttonBindings.getOperatorController();
        }
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.get();
    }

    /** This function is called once when teleop is enabled. */
    public void teleopInit() {
        m_climb.zeroPosition();
    }

    public abstract void updateMechanisms();

    // Methods used by simulation only

    // Creates the abstract method updateSimulation, which is called periodically
    // during simulation to update the state of the robot and its subsystems.
    public abstract void updateSimulation();

    // Creates the abstract method onAllianceChanged, which is called when the
    // alliance color or location changes, allowing the robot to adjust its behavior
    // accordingly.
    public abstract void onAllianceChanged(Alliance currentAlliance, int currentLocation);
}
