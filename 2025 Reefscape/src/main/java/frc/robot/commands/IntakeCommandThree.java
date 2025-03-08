package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.enums.IndexState;
import frc.robot.enums.IntakeExtentionState;
import frc.robot.enums.IntakeRollerState;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Outtake;
import frc.robot.subsystems.intake.IntakeExtention;
import frc.robot.subsystems.intake.IntakeRoller;

public class IntakeCommandThree extends SequentialCommandGroup {
    private static IntakeRoller intakeRoller = IntakeRoller.get();
    private static IntakeExtention intakeExtention = IntakeExtention.get();
    private static Index index = Index.get();
    private static Elevator elevator = Elevator.get();
    private static Outtake outtake = Outtake.get();

    public IntakeCommandThree() {
        addRequirements(intakeRoller, intakeExtention, index, elevator, outtake);

        addCommands(
                Commands.runOnce(() -> { // runs when the button is clicked, the button is clicked after the piece is
                                         // inside the outtake
                    intakeRoller.setSpeedCommand(IntakeRollerState.Stop);
                    index.updateSpeed(IndexState.Stop);
                    intakeExtention.updateCommand(IntakeExtentionState.IntakePosition);

                }));
    };
}// stop the intake rollers, stop the index, intake down