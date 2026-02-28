// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.HopperConstants;
//import frc.robot.Constants.intakeConstants;
import frc.robot.Constants.shooterConstants;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class runEverything extends SequentialCommandGroup {

    private final FeederSubsystem feederSubsystem;
    private final ShooterSubsystem shooterSubsystem;
    private final HopperSubsystem hopperSubsystem;
    // private final Intake intakeSubsystem;

    static ShuffleboardTab tab1 = Shuffleboard.getTab("feederallMotor");
    static GenericEntry feederEntry = tab1.add("SET allFEEDSPEED", shooterConstants.feederspeed).getEntry();
    static ShuffleboardTab tab2 = Shuffleboard.getTab("shooterallMotor");
    static GenericEntry shooterEntry = tab2.add("SET allSPEED", shooterConstants.speed).getEntry();
    static ShuffleboardTab tab3 = Shuffleboard.getTab("HopperallMotor");
    static GenericEntry hopperEntry = tab3.add("SET allHOPPERSPEED", HopperConstants.hopperSpeed).getEntry();
    static ShuffleboardTab tab4 = Shuffleboard.getTab("intakeallMotor");
    // static GenericEntry intakeEntry = tab4.add("SET allINTAKESPEED",
    // intakeConstants.intakeSpeed).getEntry();

    /** Creates a new runEverything. */
    public runEverything(FeederSubsystem feederSubsystem, ShooterSubsystem shooterSubsystem,
            HopperSubsystem hopperSubsystem) { // Intake intakeSubsystem{
        this.feederSubsystem = feederSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.hopperSubsystem = hopperSubsystem;
        // this.intakeSubsystem = intakeSubsystem;
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());

        addCommands(
                new InstantCommand() {
                    @Override
                    public void initialize() {
                        System.out.println("Running all commands sequentially");
                        feederSubsystem.setIdle();
                        // shooterSubsystem.setEnabled(true);
                        // shooterSubsystem.setShooterPID(shooterConstants.setVelocity);
                        hopperSubsystem.setIdle();
                    }
                },
                // new InstantCommand(() ->
                // intakeSubsystem.setIntakeSpeed(intakeEntry.getDouble(-0.1))),
                // new WaitCommand(1),
                // new shooterCommand(shooterSubsystem),
                new WaitCommand(0.5),
                new InstantCommand(() -> feederSubsystem.setFeederSpeed(feederEntry.getDouble(0.1))),
                // new WaitCommand(1),
                new InstantCommand(() -> hopperSubsystem.setSpeed(hopperEntry.getDouble(0.1))),

                new StartEndCommand(() -> {
                }, () -> {
                    feederSubsystem.setIdle();
                    // shooterSubsystem.setEnabled(false);
                    hopperSubsystem.setIdle();
                    feederSubsystem.setIdle();
                    System.out.println("All commands stopped");
                })

        );
    }

}
