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
import frc.robot.util.FuelSim;
import frc.robot.util.FuelSimCount;

/**
 * 
 */
abstract public class RobotContainer {
    /* Subsystems */
    protected SwerveDriveSubsystem swerveDriveSubsystem;
    protected VisionSubsystem visionSubsystem;
    protected ClimbSubsystem climbSubsystem;
    protected FeederSubsystem feederSubsystem;
    protected HopperSubsystem hopperSubsystem;
    protected IntakeSubsystem intakeSubsystem;
    protected ShooterSubsystem shooterSubsystem;
    protected TurretSubsystem turretSubsystem;

    /* Mechanisms */
    protected MechanismVisualizer mechanismVisualizer;

    /* Autonomous */
    protected LoggedDashboardChooser<Command> autoChooser;

    /* Controllers */
    public static CommandXboxController driverController;
    public static CommandXboxController operatorController;
    public static CommandXboxController tuningController;

    /* Simulation */
    protected FuelSim fuelSim;
    protected FuelSimCount fuelSimCount;

    /**
     * 
     */
    public RobotContainer() {

    }

    /**
     * 
     */
    abstract protected void configureAutoChooser();

    /**
     * 
     */
    protected void configureButtonBindings() {
        if (Robot.isReal()) {
            ButtonBindings buttonBindings = new ButtonBindings(swerveDriveSubsystem, climbSubsystem, feederSubsystem,
                    hopperSubsystem,
                    intakeSubsystem, shooterSubsystem, turretSubsystem);

            driverController = buttonBindings.getDriverController();
            operatorController = buttonBindings.getOperatorController();
            tuningController = buttonBindings.getTuningController();
        } else {
            ButtonsBindingsSim buttonBindings = new ButtonsBindingsSim(swerveDriveSubsystem, climbSubsystem,
                    feederSubsystem,
                    hopperSubsystem,
                    intakeSubsystem, shooterSubsystem, turretSubsystem);

            driverController = buttonBindings.getDriverController();
            operatorController = buttonBindings.getOperatorController();
            tuningController = buttonBindings.getTuningController();
        }
    }

    /**
     * 
     */
    abstract protected void configureNamedCommands();

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.get();
    }

    /** This function is called once when autonomous is enabled. */
    public void autonomousInit() {
    }

    /** This function is called once when teleop is enabled. */
    public void teleopInit() {
    }

    public abstract void updateVisualizer();

    // Methods used by simulation only

    // Creates the abstract method updateSimulation, which is called periodically
    // during simulation to update the state of the robot and its subsystems.
    public abstract void updateSimulation();

    // Creates the abstract method onAllianceChanged, which is called when the
    // alliance color or location changes, allowing the robot to adjust its behavior
    // accordingly.
    public abstract void onAllianceChanged(Alliance currentAlliance, int currentLocation);
}
