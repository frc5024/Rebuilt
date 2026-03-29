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
    private final double INTAKEARM_START_ANGLE = 0.0;

    private final LoggedMechanism2d canvas;
    private final LoggedMechanismRoot2d canvasRoot;
    private final LoggedMechanismLigament2d intakeArm;
    private final LoggedMechanismLigament2d hopperArm;
    private final LoggedMechanismLigament2d turretArm;
    private final LoggedMechanismLigament2d climbShaft;
    private final LoggedMechanismLigament2d feederArm;

    /**
     * 
     */
    public RobotMechanism() {
        this.canvas = new LoggedMechanism2d(CANVAS_SIZE, CANVAS_SIZE, new Color8Bit(Color.kWheat));
        this.canvasRoot = canvas.getRoot("InitialRoot", CANVAS_SIZE / 2, CANVAS_SIZE / 2);

        this.intakeArm = new LoggedMechanismLigament2d("IntakeArm", Units.inchesToMeters(14), INTAKEARM_START_ANGLE, 10,
                new Color8Bit(Color.kGreen));
        this.hopperArm = new LoggedMechanismLigament2d("SpindexerArm", Units.inchesToMeters(14), 0, 10,
                new Color8Bit(Color.kBlue));
        this.turretArm = new LoggedMechanismLigament2d("TurretArm", Units.inchesToMeters(14), 0, 10,
                new Color8Bit(Color.kRed));
        this.climbShaft = new LoggedMechanismLigament2d("ClimbShaft", Units.inchesToMeters(4.33), 180, 10,
                new Color8Bit(Color.kPurple));
        this.feederArm = new LoggedMechanismLigament2d("FeederArm", Units.inchesToMeters(4.33), 180, 10,
                new Color8Bit(Color.kOrange));

        this.canvasRoot.append(intakeArm);
        this.canvasRoot.append(hopperArm);
        this.canvasRoot.append(turretArm);
        this.canvasRoot.append(climbShaft);
        this.canvasRoot.append(feederArm);
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
    public double getFeederArmAngle() {
        return feederArm.getAngle();
    }

    /**
     * 
     */
    public double getHopperArmAngle() {
        return hopperArm.getAngle();
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
    public LoggedMechanism2d getMechanism() {
        return canvas;
    }

    /**
     * 
     */
    public double getTurretArmAngle() {
        return turretArm.getAngle();
    }

    /**
     * 
     */
    public void setMechanisms(double intakeArmAngle, double hopperPosition, double turretArmAngle, double climbHeight,
            double feederArmPosition) {
        intakeArm.setAngle(intakeArmAngle);
        hopperArm.setAngle(Units.radiansToDegrees(hopperPosition));
        turretArm.setAngle(turretArmAngle);
        climbShaft.setLength(Units.inchesToMeters(4.33) - climbHeight);
        feederArm.setAngle(Units.radiansToDegrees(feederArmPosition));
    }
}
