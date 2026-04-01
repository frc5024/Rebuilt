package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.statemachine.StateMachineSubsystem;
import frc.robot.Constants.ShooterConstants;

/**
 * 
 */
public class ShooterSubsystem extends StateMachineSubsystem {
    // Advantagekit logging
    private final ShooterModuleIO shooterModuleIO;
    protected final ShooterModuleIOInputsAutoLogged inputs;

    // Variables
    private double targetRPM;

    /**
     * 
     */
    public ShooterSubsystem(ShooterModuleIO shooterModuleIO) {
        super("Shooter");

        // set advantage kit IO logging
        this.shooterModuleIO = shooterModuleIO;
        this.inputs = new ShooterModuleIOInputsAutoLogged();
        this.targetRPM = 0.0;
    }

    @Override
    public void periodic() {
        super.periodic();

        // process hardware inputs
        shooterModuleIO.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);

        shooterModuleIO.setVelocity(targetRPM);

        Logger.recordOutput("Shooter/AtSetpoint", shooterModuleIO.isAtSetpoint());
        Logger.recordOutput("Shooter/CurrentVelocityRPM", getVelocity());
        Logger.recordOutput("Shooter/SetpointRPM", shooterModuleIO.getGoalVelocity());
        Logger.recordOutput("Shooter/VelocityTangential", getTangentialVelocity());
    }

    /**
     * Called from the SpinToHub to set RPM
     */
    public void addDistanceMeasurement(double distanceToTarget) {
        Command currentCommand = getCurrentCommand();

        if (currentCommand != null && currentCommand.getName().equalsIgnoreCase("ShootCommand")) {
            targetRPM = ShooterConstants.velocityToRPMMap.get(distanceToTarget);
        }
    }

    public double getCurrentDrawAmps() {
        return shooterModuleIO.getCurrentDrawAmps();
    }

    public double getPosition() {
        return shooterModuleIO.getPosition();
    }

    public double getVelocity() {
        return shooterModuleIO.getVelocity();
    }

    public double getTangentialVelocity() {
        // divide by 2 because only one side of the flywheel is touching the fuel
        // times by 0.85 for "real world" efficiency factor
        double tangentialVelocity = (getVelocity() / 60.0) * (Math.PI * ShooterConstants.WHEEL_DIAMETER_METERS);
        return tangentialVelocity / 2.0 * 0.85;
    }

    public boolean isAtSetpoint() {
        return shooterModuleIO.isAtSetpoint();
    }

    public void setVelocity(double targetRPM) {
        this.targetRPM = targetRPM;
    }

    /**
     * 
     */
    @Override
    protected void setShuffleboard() {
        this.kSVAs = ShooterConstants.getSVAs();
        this.kPIDs = ShooterConstants.getPIDs();
    }

    @Override
    protected void setShuffleboardEntries() {
        shooterModuleIO.setFF(
                sEntry.getDouble(kSVAs[0]),
                vEntry.getDouble(kSVAs[1]),
                aEntry.getDouble(kSVAs[2]));

        shooterModuleIO.setPID(
                pEntry.getDouble(kPIDs[0]),
                iEntry.getDouble(kPIDs[1]),
                dEntry.getDouble(kPIDs[2]));

        shooterModuleIO.setVelocity(rpmEntry.getDouble(0.0));
    }

    /**
     * Overrides for SysId routines
     */
    @Override
    public double getFFCharacterizationVelocity() {
        return shooterModuleIO.getFFCharacterizationVelocity();
    }

    @Override
    public void runCharacterization(double voltage) {
        shooterModuleIO.runCharacterization(voltage);
    }
}
