package frc.robot.subsystems.swervedrive;

import java.util.Queue;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.util.PhoenixOdometryThread;

/**
 * 
 */
public class GyroModuleIOPigeon2 implements GyroModuleIO {
    // Hardware
    protected final Pigeon2 pigeon;

    // Data variables
    private final StatusSignal<Angle> pitch;
    private final StatusSignal<AngularVelocity> pitchVelocity;
    private final StatusSignal<Angle> roll;
    private final StatusSignal<AngularVelocity> rollVelocity;
    private final StatusSignal<Angle> yaw;
    private final StatusSignal<AngularVelocity> yawVelocity;
    private final Queue<Double> yawPositionQueue;
    private final Queue<Double> yawTimestampQueue;

    public GyroModuleIOPigeon2() {
        this.pigeon = new Pigeon2(
                TunerConstants.DrivetrainConstants.Pigeon2Id,
                new CANBus(TunerConstants.DrivetrainConstants.CANBusName));

        Pigeon2Configuration config = new Pigeon2Configuration();
        this.pigeon.getConfigurator().refresh(config);
        // add any additional settings here
        this.pigeon.getConfigurator().apply(config);
        this.pigeon.optimizeBusUtilization();

        this.pitch = pigeon.getPitch();
        this.pitch.setUpdateFrequency(SwerveDriveConstants.ODOMETRY_FREQUENCY);

        this.pitchVelocity = pigeon.getAngularVelocityYWorld();
        this.pitchVelocity.setUpdateFrequency(50.0);

        this.roll = pigeon.getRoll();
        this.roll.setUpdateFrequency(SwerveDriveConstants.ODOMETRY_FREQUENCY);

        this.rollVelocity = pigeon.getAngularVelocityXWorld();
        this.rollVelocity.setUpdateFrequency(50.0);

        this.yaw = pigeon.getYaw();
        this.yaw.setUpdateFrequency(SwerveDriveConstants.ODOMETRY_FREQUENCY);

        this.yawVelocity = pigeon.getAngularVelocityZWorld();
        this.yawVelocity.setUpdateFrequency(50.0);

        this.yawTimestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
        this.yawPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(pigeon.getYaw());
    }

    @Override
    public void setYaw(double degrees) {
        pigeon.setYaw(degrees);
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.data = new GyroModuleIOData(
                BaseStatusSignal.refreshAll(pitch, pitchVelocity, roll, rollVelocity, yaw, yawVelocity)
                        .equals(StatusCode.OK),
                Rotation2d.fromDegrees(pitch.getValueAsDouble()),
                pitch.getValueAsDouble(),
                Units.degreesToRadians(pitchVelocity.getValueAsDouble()),
                Rotation2d.fromDegrees(roll.getValueAsDouble()),
                roll.getValueAsDouble(),
                Units.degreesToRadians(rollVelocity.getValueAsDouble()),
                Rotation2d.fromDegrees(yaw.getValueAsDouble()),
                yaw.getValueAsDouble(),
                Units.degreesToRadians(yawVelocity.getValueAsDouble()),
                yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray(),
                yawPositionQueue.stream()
                        .map((Double value) -> Rotation2d.fromDegrees(value))
                        .toArray(Rotation2d[]::new));

        yawTimestampQueue.clear();
        yawPositionQueue.clear();
    }
}
