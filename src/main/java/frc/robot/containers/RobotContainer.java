package frc.robot.containers;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Turret;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Robot;
import frc.robot.commands.feederCommand;
import frc.robot.commands.runEverything;
import frc.robot.commands.shooterCommand;
import frc.robot.commands.Hopper.Spin;
import frc.robot.commands.StickRotationCommand;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.swervedrive.SwerveDriveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * 
 */
abstract public class RobotContainer {
    /* Subsystems */
    protected SwerveDriveSubsystem swerveDriveSubsystem;
    protected VisionSubsystem visionSubsystem;

    private final Hopper m_hopper = Hopper.getInstance();
  private final Shooter m_shooter = Shooter.getInstance();
  private final Feeder m_feeder = Feeder.getInstance();
  private final Intake m_intake = Intake.getInstance();
  private final Turret m_turret = Turret.getInstance();
  

    /* Autonomous */
    protected LoggedDashboardChooser<Command> autoChooser;

    /* Controllers */
    public static CommandXboxController driverController;
    public static CommandXboxController operatorController;

private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;


    /**
     * 
     */
    protected void configureButtonBindings() {
        ButtonBindings buttonBindings = new ButtonBindings(swerveDriveSubsystem, visionSubsystem);
        driverController = buttonBindings.getDriverController();
        operatorController = buttonBindings.getOperatorController();

        driverController.a().whileTrue(m_hopper.SpinCommand());
    //driverController.b().whileTrue(m_hopper.SpinEntryCommand());
    driverController.rightBumper().whileTrue(m_shooter.shooterCommand());
    driverController.y().whileTrue(m_feeder.feederCommand());
    driverController.x().onTrue(new runEverything(m_feeder, m_shooter, m_hopper));
     
    driverController.leftBumper().whileTrue(m_feeder.jammedCommand());
  }

    

    abstract protected void configureAutoChooser();

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.get();
    }

    // Methods used by simulation only

    // Creates the abstract method updateSimulation, which is called periodically
    // during simulation to update the state of the robot and its subsystems.
    public abstract void updateSimulation();

    // Creates the abstract method onAllianceChanged, which is called when the
    // alliance color or location changes, allowing the robot to adjust its behavior
    // accordingly.
    public abstract void onAllianceChanged(Alliance currentAlliance, int currentLocation);
}
