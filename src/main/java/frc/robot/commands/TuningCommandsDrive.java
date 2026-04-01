package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.subsystems.swervedrive.SwerveDriveSubsystem;

public class TuningCommandsDrive {
    private static final double DEADBAND = 0.1;
    private static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
    private static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2

    /**
     * 
     */
    private TuningCommandsDrive() {
    }

    @SuppressWarnings("unused")
    private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
        // Apply deadband
        double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
        Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

        // Square magnitude for more precise control
        linearMagnitude = linearMagnitude * linearMagnitude;

        // Return new linear velocity
        return new Pose2d(new Translation2d(), linearDirection)
                .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                .getTranslation();
    }

    /** Measures the robot's wheel radius by spinning in a circle. */
    public static Command wheelRadiusCharacterization(SwerveDriveSubsystem swerveDriveSubsystem) {
        SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
        WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

        return Commands.parallel(
                // Drive control sequence
                Commands.sequence(
                        // Reset acceleration limiter
                        Commands.runOnce(
                                () -> {
                                    limiter.reset(0.0);
                                }),

                        // Turn in place, accelerating up to full speed
                        Commands.run(
                                () -> {
                                    double speed = limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY);
                                    swerveDriveSubsystem.runVelocity(new ChassisSpeeds(0.0, 0.0, speed));
                                },
                                swerveDriveSubsystem)),

                // Measurement sequence
                Commands.sequence(
                        // Wait for modules to fully orient before starting measurement
                        Commands.waitSeconds(1.0),

                        // Record starting measurement
                        Commands.runOnce(
                                () -> {
                                    state.positions = swerveDriveSubsystem.getWheelRadiusCharacterizationPositions();
                                    state.lastAngle = swerveDriveSubsystem.getRotation();
                                    state.gyroDelta = 0.0;
                                }),

                        // Update gyro delta
                        Commands.run(
                                () -> {
                                    var rotation = swerveDriveSubsystem.getRotation();
                                    state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                                    state.lastAngle = rotation;
                                })

                                // When cancelled, calculate and print results
                                .finallyDo(
                                        () -> {
                                            double[] positions = swerveDriveSubsystem
                                                    .getWheelRadiusCharacterizationPositions();
                                            double wheelDelta = 0.0;
                                            for (int i = 0; i < 4; i++) {
                                                wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                                            }
                                            double wheelRadius = (state.gyroDelta
                                                    * SwerveDriveConstants.DRIVE_BASE_RADIUS) / wheelDelta;

                                            // Log wheel radius characterization results
                                            Logger.recordOutput("Drive/Wheel_Radius_Characterization/wheelDelta",
                                                    wheelDelta);
                                            Logger.recordOutput("Drive/Wheel_Radius_Characterization/gyroDelta",
                                                    state.gyroDelta);
                                            Logger.recordOutput("Drive/Wheel_Radius_Characterization/wheelRadius",
                                                    wheelRadius);
                                        })));
    }

    private static class WheelRadiusCharacterizationState {
        double[] positions = new double[4];
        Rotation2d lastAngle = new Rotation2d();
        double gyroDelta = 0.0;
    }
}
