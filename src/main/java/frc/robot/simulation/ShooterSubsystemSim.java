package frc.robot.simulation;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.FuelCellConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.shooter.ShooterModuleIO;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.util.FuelSim;
import frc.robot.util.FuelSimCount;

/**
 * 
 */
public class ShooterSubsystemSim extends ShooterSubsystem {
    private final DoubleSupplier turretYawSupplier;
    private final BooleanSupplier feederIsRunningSupplier;
    private final FuelSim fuelSim;
    private final FuelSimCount fuelSimCount;

    private double lastTimestamp;

    /**
     * 
     */
    public ShooterSubsystemSim(ShooterModuleIO shooterModuleIO, DoubleSupplier turretYawSupplier,
            BooleanSupplier feederIsRunningSupplier, FuelSim fuelSim,
            FuelSimCount fuelSimCount) {
        super(shooterModuleIO);

        this.turretYawSupplier = turretYawSupplier;
        this.feederIsRunningSupplier = feederIsRunningSupplier;
        this.fuelSim = fuelSim;
        this.fuelSimCount = fuelSimCount;

        lastTimestamp = 0.0;
    }

    @Override
    public void periodic() {
        super.periodic();

        if (!isAtSetpoint() || !feederIsRunningSupplier.getAsBoolean()) {
            return;
        }

        // launch fuel based on balls per second
        if (Timer.getFPGATimestamp() - lastTimestamp < (1 / fuelSimCount.getLaunchBPS())) {
            return;
        } else {
            lastTimestamp = Timer.getFPGATimestamp();
        }

        if (fuelSimCount.getFuelStored() > 0) {
            fuelSim.launchFuel(MetersPerSecond.of(getTangentialVelocity()),
                    Degree.of(TurretConstants.verticalLaunchAngle),
                    Degree.of(turretYawSupplier.getAsDouble()),
                    Meters.of(FuelCellConstants.DIAMETER * 2.7));
            fuelSimCount.setFuelStored(fuelSimCount.getFuelStored() - 1);
        }
    }
}
