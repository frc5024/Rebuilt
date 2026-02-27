package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class IntakeDrop extends SequentialCommandGroup {

    private IntakeSubsystem s_Intake;

    public IntakeDrop(IntakeSubsystem s_Intake) { // Intake intakeSubsystem{
        this.s_Intake = s_Intake;
        // this.intakeSubsystem = intakeSubsystem;
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());

        addCommands(
                new InstantCommand() {
                    @Override
                    public void initialize() {
                        System.out.println("intake initializing");
                        s_Intake.setArmSpeed(0);
                        s_Intake.setIntakeSpeed(0);
                    }
                },

                new InstantCommand(() -> s_Intake.setArmSpeed(-0.1)),
                new WaitCommand(0.5),
                new InstantCommand(() -> s_Intake.setIntakeSpeed(0.5)),
                new StartEndCommand(() -> {
                }, () -> {
                    s_Intake.setIntakeSpeed(0);
                    s_Intake.setArmSpeed(0);
                })

        );

    }
}
