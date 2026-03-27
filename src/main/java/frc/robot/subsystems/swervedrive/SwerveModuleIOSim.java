package frc.robot.subsystems.swervedrive;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.RobotConstants;

/**
 * 
 */
public class SwerveModuleIOSim implements SwerveModuleIO {
    // Constants
    private static final double[] DRIVE_PIDs = { 0.01, 0.0, 0.0 };
    private static final double[] DRIVE_SVAs = { 0.002932, 0.14489, 0.0 };
    // private static final double[] DRIVE_SVAs = { 0.0, 0.0, 0.0 };
    private static final double[] TURN_PIDs = { 5.0, 0.0, 0.0 };
    private static final double[] TURN_SVAs = { 0.0, 0.0, 0.0 };

    // Hardware
    private final DCMotor driveDcMotor;
    private final DCMotorSim driveMotorSim;
    private PIDController driveController;

    private final DCMotor turnDcMotor;
    private final DCMotorSim turnMotorSim;
    private PIDController turnController;

    // Variables
    private boolean driveClosedLoop = false;
    private boolean turnClosedLoop = false;
    private double driveFFVolts = 0.0;
    private double driveAppliedVolts = 0.0;
    private double turnAppliedVolts = 0.0;

    /**
     * 
     */
    public SwerveModuleIOSim(
            SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> constants) {
        this.driveDcMotor = DCMotor.getKrakenX60(1);
        this.driveMotorSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(driveDcMotor, constants.DriveInertia, constants.DriveMotorGearRatio),
                driveDcMotor);

        this.driveController = new PIDController(DRIVE_PIDs[0], DRIVE_PIDs[1], DRIVE_PIDs[2]);

        this.turnDcMotor = DCMotor.getFalcon500(1);
        this.turnMotorSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(turnDcMotor, constants.SteerInertia, constants.SteerMotorGearRatio),
                turnDcMotor);

        this.turnController = new PIDController(TURN_PIDs[0], TURN_PIDs[1], TURN_PIDs[2]);

        // Enable wrapping for turn PID
        this.turnController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void updateInputs(SwerveModuleIOInputs inputs) {
        // Run closed-loop control
        if (driveClosedLoop) {
            driveAppliedVolts = driveFFVolts + driveController.calculate(driveMotorSim.getAngularVelocityRadPerSec());
        } else {
            driveController.reset();
        }
        if (turnClosedLoop) {
            turnAppliedVolts = turnController.calculate(turnMotorSim.getAngularPositionRad());
        } else {
            turnController.reset();
        }

        // Update simulation state
        driveMotorSim.setInputVoltage(MathUtil.clamp(driveAppliedVolts, -12.0, 12.0));
        turnMotorSim.setInputVoltage(MathUtil.clamp(turnAppliedVolts, -12.0, 12.0));
        driveMotorSim.update(RobotConstants.LOOP_PERIOD_SECS);
        turnMotorSim.update(RobotConstants.LOOP_PERIOD_SECS);

        // Update drive inputs
        inputs.driveConnected = true;
        inputs.drivePositionRad = driveMotorSim.getAngularPositionRad();
        inputs.driveVelocityRadPerSec = driveMotorSim.getAngularVelocityRadPerSec();
        inputs.driveAppliedVolts = driveAppliedVolts;
        inputs.driveCurrentAmps = Math.abs(driveMotorSim.getCurrentDrawAmps());

        inputs.turnConnected = true;
        inputs.turnEncoderConnected = true;
        inputs.turnAbsolutePosition = new Rotation2d(turnMotorSim.getAngularPositionRad());
        inputs.turnPosition = new Rotation2d(turnMotorSim.getAngularPositionRad());
        inputs.turnVelocityRadPerSec = turnMotorSim.getAngularVelocityRadPerSec();
        inputs.turnAppliedVolts = turnAppliedVolts;
        inputs.turnCurrentAmps = Math.abs(turnMotorSim.getCurrentDrawAmps());

        // Update odometry inputs (50Hz because high-frequency odometry in sim doesn't
        // matter)
        inputs.odometryTimestamps = new double[] { Timer.getTimestamp() };
        inputs.odometryDrivePositionsRad = new double[] { inputs.drivePositionRad };
        inputs.odometryTurnPositions = new Rotation2d[] { inputs.turnPosition };
    }

    @Override
    public double getCurrentDrawAmps() {
        return Math.abs(driveMotorSim.getCurrentDrawAmps()) + Math.abs(turnMotorSim.getCurrentDrawAmps());
    }

    @Override
    public void setDriveOpenLoop(double output) {
        driveClosedLoop = false;
        driveAppliedVolts = output;
    }

    @Override
    public void setTurnOpenLoop(double output) {
        turnClosedLoop = false;
        turnAppliedVolts = output;
    }

    @Override
    public void setDriveVelocity(double velocityRadPerSec) {
        driveClosedLoop = true;
        driveFFVolts = DRIVE_SVAs[0] * Math.signum(velocityRadPerSec) + DRIVE_SVAs[1] * velocityRadPerSec;
        driveController.setSetpoint(velocityRadPerSec);
    }

    @Override
    public void setTurnPosition(Rotation2d rotation) {
        turnClosedLoop = true;
        turnController.setSetpoint(rotation.getRadians());
    }
}
