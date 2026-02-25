package frc.robot.simulation;

import frc.robot.subsystems.shooter.ShooterModuleIO;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.util.MapleSimUtil;

/**
 * 
 */
public class ShooterSubsystemSim extends ShooterSubsystem {
    /**
     * 
     */
    public ShooterSubsystemSim(ShooterModuleIO shooterModuleIO) {
        super(shooterModuleIO);
    }

    @Override
    public void setPIDMotor() {
        super.setPIDMotor();

        MapleSimUtil.ejectFuelFromRobot();
    }
}
