package frc.robot.commands.Turret;

import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.containers.RobotContainer;
import frc.robot.subsystems.swervedrive.SwerveDriveSubsystem;
import frc.robot.subsystems.Turret;
import frc.robot.Constants;

public class LockSetpointCommand extends Command {

    private final Turret turretSubsystem;
    private final SwerveDriveSubsystem swerveDriveSubsystem;
    //private final double targetAngle;

    // CommandXboxController operator = RobotContainer.operator;

    public LockSetpointCommand(Turret turretSubsystem, SwerveDriveSubsystem swerveDriveSubsystem) {
        this.turretSubsystem = turretSubsystem;
        this.swerveDriveSubsystem = swerveDriveSubsystem;
        addRequirements(turretSubsystem, swerveDriveSubsystem);
    }

    public void initialize() {
        System.out.println("setting turret angle");
        // Reset PID to the current turret position
        turretSubsystem.resetPID(); // turretSubsystem.zeroEncoder();

        // Desired field direction
        double desiredFieldAngle = 90.0; 

        // Get robot heading from odometry (degrees)
        double robotHeading = swerveDriveSubsystem.getRotation().getDegrees();

        double turretTargetField = desiredFieldAngle - robotHeading;

        // Normalize to [-180, 180)
        while (turretTargetField >= 180.0) {
            turretTargetField -= 360.0;
        }
        while (turretTargetField < -180.0) {
            turretTargetField += 360.0;
        }

        // Respect turret free-rotation of 270 degrees by clamping required motion to +/-135 deg
        double currentTurretAngle = turretSubsystem.getTurretAngle();

        // Compute shortest angle (desired - current) 
        double delta = turretTargetField - currentTurretAngle;
        while (delta >= 180.0) {
            delta -= 360.0;
        }
        while (delta < -180.0) {
            delta += 360.0;
        }

        final double halfRange = 135.0;
        double clampedDelta = delta;
        if (Math.abs(delta) > halfRange) {
            clampedDelta = Math.copySign(halfRange, delta); //if delta >= 0 clamped = 135, if delta =< 0 clamped = -135
            System.out.println(String.format("Desired delta %.2f exceeds half-range %.2f; clamping to %.2f", delta, halfRange, clampedDelta));
        }

        double reachableTarget = currentTurretAngle + clampedDelta;

        // Normalize reachable target into [-180,180)
        while (reachableTarget >= 180.0) {
            reachableTarget -= 360.0;
        }
        while (reachableTarget < -180.0) {
            reachableTarget += 360.0;
        }

        System.out.println(String.format("Locking turret to field angle %.2f (robot heading %.2f) -> desired %.2f -> reachable %.2f", desiredFieldAngle, robotHeading, turretTargetField, reachableTarget));

        // Set the turret setpoint and enable PID control
        turretSubsystem.setTargetAngle(reachableTarget);
        turretSubsystem.enablePID();
    }

    public void execute() {
        // System.out.println(turretSubsystem.getTurretAngle());
        // turretSubsystem.updateTurretAngle();

    }

    public boolean isFinished() {
        if (turretSubsystem.isAtTargetAngle() == true) {
            System.out.println("AT SETPOINT");
        }
        return turretSubsystem.isAtTargetAngle();
    }

    @Override
    public void end(boolean interrupted) {
        turretSubsystem.disablePID();
        // Optionally stop motor explicitly
        turretSubsystem.setIdle();
    }

}
