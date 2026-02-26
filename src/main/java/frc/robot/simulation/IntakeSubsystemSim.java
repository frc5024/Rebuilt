package frc.robot.simulation;

import org.littletonrobotics.junction.Logger;

import frc.robot.subsystems.intake.IntakeModuleIO;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.util.MapleSimUtil;

/**
 * 
 */
public class IntakeSubsystemSim extends IntakeSubsystem {
    /**
     * 
     */
    public IntakeSubsystemSim(IntakeModuleIO intakeModuleIO) {
        super(intakeModuleIO);
    }

    @Override
    public void periodic() {
        super.periodic();

        Logger.recordOutput("FieldSimulation/Intake Fuel Poses", MapleSimUtil.getFuelPoses());
    }

    @Override
    public void setIntakeSpeed(double speed) {
        super.setIntakeSpeed(speed);

        if (speed > 0.0) {
            MapleSimUtil.getIntakeSimulation().startIntake();
        } else {
            MapleSimUtil.getIntakeSimulation().stopIntake();
        }
    }
}
