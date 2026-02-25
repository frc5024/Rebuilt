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
import frc.robot.Constants.shooterConstants;

/**
 * 
 */
public class ShooterSubsystem extends SubsystemBase {
    private final ShooterModuleIO shooterModuleIO;
    protected final ShooterModuleIOInputsAutoLogged inputs;

    private boolean enabled;

    private PIDController PID;
    private SimpleMotorFeedforward feedForward;

    ShuffleboardTab tab = Shuffleboard.getTab("Shooter");
    GenericEntry pEntry = tab.add("SET P", shooterConstants.kP).getEntry();
    GenericEntry dEntry = tab.add("SET D", shooterConstants.kD).getEntry();
    GenericEntry iEntry = tab.add("SET I", shooterConstants.kI).getEntry();
    GenericEntry sEntry = tab.add("SET S", shooterConstants.kS).getEntry();
    GenericEntry vEntry = tab.add("SET V", shooterConstants.kV).getEntry();
    GenericEntry aEntry = tab.add("SET A", shooterConstants.kA).getEntry();
    GenericEntry setVelocityEntry = tab.add("SET VELOCITY", shooterConstants.setVelocity).getEntry();

    /**
     * 
     */
    public ShooterSubsystem(ShooterModuleIO shooterModuleIO) {
        this.shooterModuleIO = shooterModuleIO;
        this.inputs = new ShooterModuleIOInputsAutoLogged();

        PID = new PIDController(shooterConstants.kP, shooterConstants.kI, shooterConstants.kD);

        feedForward = new SimpleMotorFeedforward(shooterConstants.kS, shooterConstants.kV, shooterConstants.kA);
    }

    @Override
    public void periodic() {
        shooterModuleIO.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);

        if (enabled) {
            setPIDMotor();

        } else {
            shooterModuleIO.set(0);
        }

    }

    public void setShooterPID(double setVelocity) {
        PID.setSetpoint(setVelocity);

    }

    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
    }

    public void setPIDMotor() {

        double setVelocity = setVelocityEntry.getDouble(100);
        PID.setSetpoint(setVelocity);

        PID.setP(pEntry.getDouble(shooterConstants.kP));
        PID.setI(iEntry.getDouble(shooterConstants.kI));
        PID.setD(dEntry.getDouble(shooterConstants.kD));

        feedForward.setKs(sEntry.getDouble(shooterConstants.kS));
        feedForward.setKv(vEntry.getDouble(shooterConstants.kV));
        feedForward.setKa(aEntry.getDouble(shooterConstants.kA));

        double PIDoutput = PID.calculate(shooterModuleIO.getVelocity());
        double feedForwardOutput = feedForward.calculate(PID.getSetpoint());
        double totalOutput = PIDoutput + feedForwardOutput;

        shooterModuleIO.setVoltage(totalOutput);

        SmartDashboard.putNumber("PID", PIDoutput);
        SmartDashboard.putNumber("FeedForward", feedForwardOutput);
        SmartDashboard.putNumber("Total Output", totalOutput);

    }

    // public Pose2d getPosition() {
    // return frc.robot.subsystems.swervedrive.SwerveDriveSubsystem.getPose();
    // }

    public Command shooterCommand() {

        return new frc.robot.commands.shooterCommand(this, shooterConstants.setVelocity);
    }

    // public Command runEverything() {
    // return new frc.robot.commands.runEverything(this,
    // shooterConstants.setVelocity);
    // }

}
