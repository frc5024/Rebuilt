package frc.robot.containers;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Robot;
import frc.robot.commands.runEverything;
import frc.robot.commands.shooterCommand;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.shooter;
import frc.robot.subsystems.swervedrive.SwerveDriveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import frc.robot.containers.ButtonBindings;

/**
 * 
 */
abstract public class RobotContainer {
    /* Subsystems */
    protected SwerveDriveSubsystem swerveDriveSubsystem;
    protected VisionSubsystem visionSubsystem;

  private final shooter m_shooter = shooter.getInstance();
  //private final Intake m_intake = Intake.getInstance();
  private final Hopper m_hopper = Hopper.getInstance();
  private final Feeder m_feeder = Feeder.getInstance();
  private final Turret m_turret = Turret.getInstance();
  private final Intake m_intake = Intake.getInstance();
  

    /* Autonomous */
    protected LoggedDashboardChooser<Command> autoChooser;

    /* Controllers */
    public static CommandXboxController driverController;
    public static CommandXboxController operatorController;

    private final shooter shooterSubsystem = new shooter();
    
    /**
     * 
     */
    protected void configureButtonBindings() {
        ButtonBindings buttonBindings = new ButtonBindings(swerveDriveSubsystem, visionSubsystem);
        driverController = buttonBindings.getDriverController();
        operatorController = buttonBindings.getOperatorController();
    }


    /**
     * 
     */
    protected void configureButtonBindings() {
        ButtonBindings buttonBindings = new ButtonBindings(swerveDriveSubsystem, visionSubsystem);
        driverController = buttonBindings.getDriverController();
        operatorController = buttonBindings.getOperatorController();

    //driverController.b().whileTrue(m_hopper.SpinEntryCommand());
    driverController.rightBumper().whileTrue(m_shooter.shooterCommand());
    driverController.y().whileTrue(m_feeder.feederCommand());
    driverController.a().whileTrue(m_hopper.SpinCommand());
    driverController.x().onTrue(new runEverything(m_feeder, m_shooter, m_hopper));
    driverController.povRight().whileTrue(m_turret.stickRotation(0.1));
    driverController.povLeft().whileTrue(m_turret.stickRotation(-0.1));
    driverController.rightTrigger().whileTrue(m_intake.ExtendSpin());
    driverController.leftTrigger().whileTrue(m_intake.RetractSpin());
    driverController.b().whileTrue(m_intake.IntakeSpin());

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
