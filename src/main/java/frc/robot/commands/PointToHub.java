package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PIDConstants;
import frc.robot.subsystems.swervedrive.SwerveDriveSubsystem;
import frc.robot.util.GameUtil;

public class PointToHub extends Command {
    private final SwerveDriveSubsystem swerveDriveSubsystem;

    private final PIDController omegaController;

    ShuffleboardTab tab = Shuffleboard.getTab("Turret");
    GenericEntry pEntry = tab.add("SET P", PIDConstants.kP).getEntry();
    GenericEntry dEntry = tab.add("SET D", PIDConstants.kD).getEntry();
    GenericEntry iEntry = tab.add("SET I", PIDConstants.kI).getEntry();

    public PointToHub(SwerveDriveSubsystem swerveDriveSubsystem, PIDController omegaController) {
        this.swerveDriveSubsystem = swerveDriveSubsystem;
        addRequirements(swerveDriveSubsystem);

        this.omegaController = new PIDController(PIDConstants.kP, PIDConstants.kI, PIDConstants.kD);

        this.omegaController.enableContinuousInput(-Math.PI, Math.PI);

        pEntry.setDouble(PIDConstants.kP);
        iEntry.setDouble(PIDConstants.kI);
        dEntry.setDouble(PIDConstants.kD);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

        // swerveDriveSubsystem.setHeading(angleToHub);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        Pose2d robotPose = swerveDriveSubsystem.getPose();
        double robotX = robotPose.getX();
        double robotY = robotPose.getY();

        Pose2d hubPose = GameUtil.getHubPose();
        double hubX = hubPose.getX();
        double hubY = hubPose.getY();

        // double omegaSpeed =
        // this.omegaController.calculate(robotPose.getRotation().getRadians());

        double angleToHub = Math.atan2(hubY - robotY, hubX - robotX);

        omegaController.setSetpoint(angleToHub);

        omegaController.setP(pEntry.getDouble(PIDConstants.kP));
        omegaController.setI(iEntry.getDouble(PIDConstants.kI));
        omegaController.setD(dEntry.getDouble(PIDConstants.kD));

        double omegaValue = omegaController.calculate(swerveDriveSubsystem.getPose().getRotation().getRadians());

        // ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(0, 0,
        // omegaValue,

        // swerveDriveSubsystem.getPose().getRotation());
        // this.swerveDriveSubsystem.drive(chassisSpeeds);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

    }

}
