// package frc.robot.commands;

// import java.lang.annotation.ElementType;

// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.robot.enums.ElevatorState;
// import frc.robot.enums.IntakeExtentionState;
// import frc.robot.subsystems.Elevator;
// import frc.robot.subsystems.Index;
// import frc.robot.subsystems.intake.IntakeExtention;
// import frc.robot.subsystems.intake.IntakeRoller;

// public class CoralPathCommand extends SequentialCommandGroup {
// private final Elevator elevator = Elevator.get();
// private final IntakeRoller intakeRoller = IntakeRoller.get();
// private final IntakeExtention intakeExtention = IntakeExtention.get();
// private final Index index = Index.get();

// public CoralPathCommand() {
// System.out.println("Coral Path Command Scheduled");
// intakeExtention.updateCommand(IntakeExtentionState.IntakePosition);
// elevator.updateCommand(ElevatorState.Down);

// }

// }
