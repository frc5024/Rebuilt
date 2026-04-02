package frc.robot.subsystems.swervedrive;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * 
 */
public interface GyroModuleIO {
    @AutoLog
    public static class GyroIOInputs {
        public GyroModuleIOData data = new GyroModuleIOData(false, new Rotation2d(), 0.0, 0.0, new Rotation2d(), 0.0,
                0.0, new Rotation2d(), 0.0, 0.0, new double[] {}, new Rotation2d[] {});
    }

    record GyroModuleIOData(
            boolean connected,
            Rotation2d pitchPosition,
            double pitchDegrees,
            double pitchVelocityRadPerSec,
            Rotation2d rollPosition,
            double rollDegrees,
            double rollVelocityRadPerSec,
            Rotation2d yawPosition,
            double yawDegrees,
            double yawVelocityRadPerSec,
            double[] odometryYawTimestamps,
            Rotation2d[] odometryYawPositions) {
    }

    default void updateInputs(GyroIOInputs inputs) {
    }
}
