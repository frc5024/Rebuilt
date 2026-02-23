package frc.robot.mechanisms;

import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

/**
 * 
 */
public class RobotMechanism {
    private final double CANVAS_SIZE = Units.inchesToMeters(90);

    private final LoggedMechanism2d canvas;
    private final LoggedMechanismRoot2d canvasRoot;
    private final LoggedMechanismLigament2d intakeArm;
    private final LoggedMechanismLigament2d climbShaft;
    private final LoggedMechanismLigament2d shooterArm;
    private final LoggedMechanismLigament2d spindexerArm;

    /**
     * 
     */
    public RobotMechanism() {
        this.canvas = new LoggedMechanism2d(CANVAS_SIZE, CANVAS_SIZE, new Color8Bit(Color.kWheat));
        this.canvasRoot = canvas.getRoot("InitialRoot", CANVAS_SIZE / 2, 0);

        this.intakeArm = new LoggedMechanismLigament2d("IntakeArm", Units.inchesToMeters(14), 90, 10,
                new Color8Bit(Color.kGreen));
        this.climbShaft = new LoggedMechanismLigament2d("ClimbShaft", Units.inchesToMeters(4.33), 180, 10,
                new Color8Bit(Color.kPurple));
        this.shooterArm = new LoggedMechanismLigament2d("ShooterArm", Units.inchesToMeters(14), 0, 10,
                new Color8Bit(Color.kRed));
        this.spindexerArm = new LoggedMechanismLigament2d("SpindexerArm", Units.inchesToMeters(14), 0, 10,
                new Color8Bit(Color.kBlue));

        this.canvasRoot.append(intakeArm);
        this.canvasRoot.append(climbShaft);
        this.canvasRoot.append(shooterArm);
        this.canvasRoot.append(spindexerArm);
    }

    /**
     * 
     */
    public double getIntakeArmAngle() {
        return intakeArm.getAngle();
    }

    /**
     * 
     */
    public double getClimbShaftLength() {
        return climbShaft.getLength();
    }

    /**
     * 
     */
    public double getShooterArmAngle() {
        return shooterArm.getAngle();
    }

    /**
     * 
     */
    public double getSpindexerArmAngle() {
        return spindexerArm.getAngle();
    }

    /**
     * 
     */
    public void setMechanisms(double armAngle, double climbHeight, double shooterAngle, double spindexerAngle) {
        // Logger.recordOutput("ArmAngle", armAngle);
        // Logger.recordOutput("ClimbHeight", climbHeight);
        // Logger.recordOutput("ShooterAngle", shooterAngle);
        // Logger.recordOutput("SpindexerAngle", spindexerAngle);

        intakeArm.setAngle(armAngle);
        climbShaft.setLength(Units.inchesToMeters(4.33) - climbHeight);
        shooterArm.setAngle(shooterAngle);
        spindexerArm.setAngle(spindexerAngle);
    }
}
