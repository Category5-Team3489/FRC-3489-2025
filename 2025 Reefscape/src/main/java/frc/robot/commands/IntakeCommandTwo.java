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

public class IntakeCommandTwo extends SequentialCommandGroup {
    private static IntakeRoller intakeRoller = IntakeRoller.get();
    private static IntakeExtention intakeExtention = IntakeExtention.get();
    private static Index index = Index.get();
    private static Elevator elevator = Elevator.get();
    private static Outtake outtake = Outtake.get();

    public IntakeCommandTwo() {
        addRequirements(intakeRoller, intakeExtention, index, elevator, outtake);

        addCommands(
                Commands.runOnce(() -> { // runs when the button is clicked, the button is clicked after the starting
                                         // positions are set
                    // intakeRoller.setSpeedCommand(IntakeRollerState.IntakeCollect); // these
                    // should all run after the
                    // previous command finishes
                    // intakeExtention.updateCommand(IntakeExtentionState.HomePosition);
                    intakeRoller.setSpeedCommand(IntakeRollerState.IntakeTransfer).schedule();
                    index.updateSpeed(IndexState.Outtake).schedule();
                }));
    }
}
// start rollers (it should automactically stop when piece is inside), intake
// completely up, start rollers (again to push into the index), start index (it
// goes all the way inside outtake)