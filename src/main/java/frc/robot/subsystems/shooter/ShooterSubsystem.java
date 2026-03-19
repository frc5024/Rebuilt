package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.shooterCommand;

/**
 * 
 */
public class ShooterSubsystem extends SubsystemBase {
    // Advantagekit logging
    private final ShooterModuleIO shooterModuleIO;
    protected final ShooterModuleIOInputsAutoLogged inputs;

    // Shuffleboard entries
    private ShuffleboardTab tab;
    private GenericEntry pEntry;
    private GenericEntry dEntry;
    private GenericEntry iEntry;

    private GenericEntry sEntry;
    private GenericEntry vEntry;
    private GenericEntry aEntry;
    private GenericEntry rpmEntry;

    /**
     * 
     */
    public ShooterSubsystem(ShooterModuleIO shooterModuleIO) {
        // set advantage kit IO logging
        this.shooterModuleIO = shooterModuleIO;
        this.inputs = new ShooterModuleIOInputsAutoLogged();

        // set shuffleboard entries if in tuning mode
        if (RobotConstants.TUNING_MODE) {
            setShuffleboard();
        }
    }

    @Override
    public void periodic() {
        // process hardware inputs
        shooterModuleIO.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);

        // update pid values if in tuning mode
        if (RobotConstants.TUNING_MODE) {
            shooterModuleIO.setPID(
                    pEntry.getDouble(ShooterConstants.kP),
                    iEntry.getDouble(ShooterConstants.kI),
                    dEntry.getDouble(ShooterConstants.kD));

            shooterModuleIO.setFF(
                    sEntry.getDouble(ShooterConstants.kS),
                    vEntry.getDouble(ShooterConstants.kV),
                    aEntry.getDouble(ShooterConstants.kA));

            setVelocity(rpmEntry.getDouble(getVelocity()));
        }

        Logger.recordOutput("Shooter/AtSetpoint", shooterModuleIO.isAtSetpoint());
        Logger.recordOutput("Shooter/CurrentVelocityRPM", getVelocity());
        Logger.recordOutput("Shooter/SetpointRPM", shooterModuleIO.getGoalVelocity());
        Logger.recordOutput("Shooter/VelocityTangential", getTangentialVelocity());
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

    public void setShooterPID(double setVelocity) {
    }

    public void setPidEnabled(boolean enabled) {
    }

    public void setVelocity(double rpm) {
        shooterModuleIO.setVelocity(rpm);
    }

    /**
     * 
     */
    private void setShuffleboard() {
        tab = Shuffleboard.getTab("Shooter");

        double[] kPIDs = ShooterConstants.getPIDs();
        pEntry = tab.add("SET P", kPIDs[0]).getEntry();
        iEntry = tab.add("SET I", kPIDs[1]).getEntry();
        dEntry = tab.add("SET D", kPIDs[2]).getEntry();

        double[] kSVAs = ShooterConstants.getSVAs();
        sEntry = tab.add("SET S", kSVAs[0]).getEntry();
        vEntry = tab.add("SET V", kSVAs[1]).getEntry();
        aEntry = tab.add("SET A", kSVAs[2]).getEntry();

        rpmEntry = tab.add("SET RPM", 0.0).getEntry();

        pEntry.setDouble(kPIDs[0]);
        iEntry.setDouble(kPIDs[1]);
        dEntry.setDouble(kPIDs[2]);

        sEntry.setDouble(kSVAs[0]);
        vEntry.setDouble(kSVAs[1]);
        aEntry.setDouble(kSVAs[2]);

        rpmEntry.setDouble(0.0);
    }

    /**
     * Commands
     */

    public Command shooterCommand() {
        return new shooterCommand(this, ShooterConstants.setVelocity);
    }
}
