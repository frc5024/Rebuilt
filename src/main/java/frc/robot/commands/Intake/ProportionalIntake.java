package frc.robot.commands.Intake;

import java.util.function.Supplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.intakeConstants;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class ProportionalIntake extends Command {
    private final IntakeSubsystem s_Intake;
    private final Supplier<ChassisSpeeds> chassisSpeedSupplier;

    static ShuffleboardTab tab = Shuffleboard.getTab("intakeMotor");
    static GenericEntry intakeEntry = tab.add("SET INTAKESPEED", intakeConstants.INTAKE_SPEED).getEntry();
    static GenericEntry coefficientEntry = tab.add("coefficient", intakeConstants.driveCoefficient).getEntry();

    public ProportionalIntake(IntakeSubsystem s_Intake, Supplier<ChassisSpeeds> chassisSpeedSupplier) {
        this.s_Intake = s_Intake;
        this.chassisSpeedSupplier = chassisSpeedSupplier;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        ChassisSpeeds chassisSpeeds = chassisSpeedSupplier.get();

        // shuffleboard tabs
        double coefficient = chassisSpeeds.vxMetersPerSecond * 0.1;
        coefficientEntry.setDouble(coefficient);

        double proportionalSpeed = intakeEntry.getDouble(intakeConstants.INTAKE_SPEED) * coefficient;
        tab.add("proportional speed", proportionalSpeed);

        s_Intake.setIntakeSpeed(proportionalSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        s_Intake.setIntakeSpeed(0.0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
