package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.shooterConstants;
import frc.robot.commands.shooterCommand;

/**
 * 
 */
public class ShooterSubsystem extends SubsystemBase {
    // Advantagekit logging
    private final ShooterModuleIO shooterModuleIO;
    protected final ShooterModuleIOInputsAutoLogged inputs;

    // PID
    private PIDController PID;
    private SimpleMotorFeedforward feedForward;

    // Shuffleboard entries
    private ShuffleboardTab tab;
    private GenericEntry pEntry;
    private GenericEntry dEntry;
    private GenericEntry iEntry;
    private GenericEntry sEntry;
    private GenericEntry vEntry;
    private GenericEntry aEntry;
    private GenericEntry setVelocityEntry;

    // Variables
    protected boolean enabled;

    /**
     * 
     */
    public ShooterSubsystem(ShooterModuleIO shooterModuleIO) {
        // set advantage kit IO logging
        this.shooterModuleIO = shooterModuleIO;
        this.inputs = new ShooterModuleIOInputsAutoLogged();

        this.PID = new PIDController(shooterConstants.kP, shooterConstants.kI, shooterConstants.kD);
        this.feedForward = new SimpleMotorFeedforward(shooterConstants.kS, shooterConstants.kV, shooterConstants.kA);

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

        if (enabled) {
            setPIDMotor();

        } else {
            shooterModuleIO.set(0);
        }

        Logger.recordOutput("Shooter/Enabled", enabled);
        Logger.recordOutput("Shooter/AtSetpoint", shooterModuleIO.isAtSetpoint());
        Logger.recordOutput("Shooter/SetpointRPM", PID.getSetpoint());
        Logger.recordOutput("Shooter/VelocityTangential", getTangentialVelocity());
        Logger.recordOutput("Shooter/VelocityRPM", getVelocity());
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
        double tangentialVelocity = (getVelocity() / 60.0) * (Math.PI * shooterConstants.WHEEL_DIAMETER_METERS);
        return tangentialVelocity / 2.0 * 0.85;
    }

    public void setShooterPID(double setVelocity) {
        PID.setSetpoint(setVelocity);
    }

    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
    }

    public void setPIDMotor() {
        // double setVelocity = setVelocityEntry.getDouble(100);
        // PID.setSetpoint(setVelocity);

        // update pid values if in tuning mode
        if (RobotConstants.TUNING_MODE) {
            PID.setP(pEntry.getDouble(shooterConstants.kP));
            PID.setI(iEntry.getDouble(shooterConstants.kI));
            PID.setD(dEntry.getDouble(shooterConstants.kD));

            feedForward.setKs(sEntry.getDouble(shooterConstants.kS));
            feedForward.setKv(vEntry.getDouble(shooterConstants.kV));
            feedForward.setKa(aEntry.getDouble(shooterConstants.kA));
        }

        double PIDoutput = PID.calculate(shooterModuleIO.getVelocity());
        double feedForwardOutput = feedForward.calculate(PID.getSetpoint());
        double totalOutput = PIDoutput + feedForwardOutput;

        shooterModuleIO.setVoltage(totalOutput);

        SmartDashboard.putNumber("PID", PIDoutput);
        SmartDashboard.putNumber("FeedForward", feedForwardOutput);
        SmartDashboard.putNumber("Total Output", totalOutput);

        Logger.recordOutput("Shooter/PID", PIDoutput);
        Logger.recordOutput("Shooter/FeedForward", feedForwardOutput);
    }

    // public Pose2d getPosition() {
    // return frc.robot.subsystems.swervedrive.SwerveDriveSubsystem.getPose();
    // }

    public void setVelocity(double rpm) {
        shooterModuleIO.setVelocity(rpm);
    }

    public Command shooterCommand() {
        return new shooterCommand(this, shooterConstants.setVelocity);
    }

    /**
     * 
     */
    private void setShuffleboard() {
        tab = Shuffleboard.getTab("Shooter");
        pEntry = tab.add("SET P", shooterConstants.kP).getEntry();
        dEntry = tab.add("SET D", shooterConstants.kD).getEntry();
        iEntry = tab.add("SET I", shooterConstants.kI).getEntry();
        sEntry = tab.add("SET S", shooterConstants.kS).getEntry();
        vEntry = tab.add("SET V", shooterConstants.kV).getEntry();
        aEntry = tab.add("SET A", shooterConstants.kA).getEntry();
        setVelocityEntry = tab.add("SET VELOCITY", shooterConstants.setVelocity).getEntry();
    }
}
