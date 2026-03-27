package frc.robot.simulation;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.FuelCellConstants;
import frc.robot.Constants.turretConstants;
import frc.robot.subsystems.shooter.ShooterModuleIO;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.util.FuelSim;
import frc.robot.util.FuelSimCount;

/**
 * 
 */
public class ShooterSubsystemSim extends ShooterSubsystem {
    private final FuelSim fuelSim;
    private final FuelSimCount fuelSimCount;

    private double lastTimestamp;

    /**
     * 
     */
    public ShooterSubsystemSim(ShooterModuleIO shooterModuleIO, FuelSim fuelSim, FuelSimCount fuelSimCount) {
        super(shooterModuleIO);

        this.fuelSim = fuelSim;
        this.fuelSimCount = fuelSimCount;

        lastTimestamp = 0.0;
    }

    @Override
    public void periodic() {
        super.periodic();

        if (!enabled) {
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
                    Degree.of(turretConstants.verticalLaunchAngle),
                    Degree.of(0.0),
                    Meters.of(FuelCellConstants.DIAMETER * 2.7));
            fuelSimCount.setFuelStored(fuelSimCount.getFuelStored() - 1);
        }
    }
}
