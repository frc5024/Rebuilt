package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.turretConstants;

/**
 * 
 */
public class TurretSubsystem extends SubsystemBase {
    private final TurretModuleIO turretModuleIO;
    protected final TurretModuleIOInputsAutoLogged inputs;

    ShuffleboardTab tab = Shuffleboard.getTab("Turret");
    GenericEntry pEntry = tab.add("SET P", turretConstants.kP).getEntry();
    GenericEntry dEntry = tab.add("SET D", turretConstants.kD).getEntry();
    GenericEntry iEntry = tab.add("SET I", turretConstants.kI).getEntry();

    GenericEntry sEntry = tab.add("SET S", turretConstants.kS).getEntry();
    GenericEntry vEntry = tab.add("SET V", turretConstants.kV).getEntry();
    GenericEntry aEntry = tab.add("SET A", turretConstants.kA).getEntry();

    GenericEntry maxSpeedEntry = tab.add("SET max speed", turretConstants.turretMaxSpeed).getEntry();
    GenericEntry maxAccelEntry = tab.add("SET max accel", turretConstants.turretMaxAccel).getEntry();
    GenericEntry toleranceEntry = tab.add("SET TOLERANCE", turretConstants.turretTolerance).getEntry();

    /**
     * 
     */
    public TurretSubsystem(TurretModuleIO turretModuleIO) {
        this.turretModuleIO = turretModuleIO;
        this.inputs = new TurretModuleIOInputsAutoLogged();

        // tab.addDouble("current angle", () -> getCurrentAngle());
        // tab.addDouble("goal", () -> turretModuleIO.getGoal().position);
        // tab.addDouble("current velocity", () -> turretModuleIO.getVelocity());
        // tab.addBoolean("pid enabled", () -> pidEnabled);
        // tab.addDouble("voltage value", () -> voltageValue);
        // tab.addDouble("pid value", () -> pValue);
        // tab.addDouble("ff value", () -> fValue);
        // tab.addBoolean("at target", () -> isAtTargetAngle());
        // tab.addDouble("Estimated Velocity", () ->
        // pidController.getSetpoint().velocity);

        pEntry.setDouble(Constants.turretConstants.kP);
        iEntry.setDouble(Constants.turretConstants.kI);
        dEntry.setDouble(Constants.turretConstants.kD);
        vEntry.setDouble(Constants.turretConstants.kV);
        maxSpeedEntry.setDouble(Constants.turretConstants.turretMaxSpeed);
        maxAccelEntry.setDouble(Constants.turretConstants.turretMaxAccel);
        toleranceEntry.setDouble(Constants.turretConstants.turretTolerance);
    }

    @Override
    public void periodic() {
        turretModuleIO.updateInputs(inputs);
        Logger.processInputs("Turret", inputs);

        turretModuleIO.setPID(
                pEntry.getDouble(turretConstants.kP),
                iEntry.getDouble(turretConstants.kI),
                dEntry.getDouble(turretConstants.kD));

        turretModuleIO.setFF(
                sEntry.getDouble(turretConstants.kS),
                vEntry.getDouble(turretConstants.kV),
                aEntry.getDouble(turretConstants.kA));

        Logger.recordOutput("Turret/CurrentAngle", getCurrentAngle());
        Logger.recordOutput("Turret/SetPointAngle", turretModuleIO.getGoalPosition());
    }

    public double getCurrentDrawAmps() {
        return turretModuleIO.getCurrentDrawAmps();
    }

    public double getCurrentAngle() {
        return turretModuleIO.getCurrentAngle();
    }

    public void decreaseAngle() {
        turretModuleIO.setAngle(getCurrentAngle() - 1);
    }

    public void increaseAngle() {
        turretModuleIO.setAngle(getCurrentAngle() + 1);
    }

    public void setAngle(double degrees) {
        turretModuleIO.setAngle(degrees);
    }

    public void zeroEncoder() {
        turretModuleIO.setPosition(0.0);
    }
}
