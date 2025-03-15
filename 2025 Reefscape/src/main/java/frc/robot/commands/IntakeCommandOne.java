// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.robot.enums.ElevatorState;
// import frc.robot.enums.IndexState;
// import frc.robot.enums.IntakeExtentionState;
// import frc.robot.enums.IntakeRollerState;
// import frc.robot.enums.OuttakeState;
// import frc.robot.subsystems.Elevator;
// import frc.robot.subsystems.Index;
// import frc.robot.subsystems.Outtake;
// import frc.robot.subsystems.intake.IntakeExtention;
// import frc.robot.subsystems.intake.IntakeRoller;

// public class IntakeCommandOne extends SequentialCommandGroup {
//     private static IntakeRoller intakeRoller = IntakeRoller.get();
//     private static IntakeExtention intakeExtention = IntakeExtention.get();
//     private static Index index = Index.get();
//     private static Elevator elevator = Elevator.get();
//     private static Outtake outtake = Outtake.get();

//     public IntakeCommandOne() {
//         addRequirements(intakeRoller, intakeExtention, index, elevator, outtake);
//         System.out.println("INIT One");

//         addCommands(
//                 Commands.runOnce(() -> {
//                     System.out.println("START ONE");
//                     outtake.updateSpeed(OuttakeState.Stop).schedule();
//                     index.updateSpeed(IndexState.Stop).schedule();
//                     intakeExtention.updateCommand(IntakeExtentionState.IntakePosition).schedule();
//                     intakeRoller.setSpeedCommand(IntakeRollerState.IntakeTransfer).schedule();
//                     elevator.updateCommand(ElevatorState.Down).schedule();
//                     // System.out.println("END ONE");
//                 }));
//     }

// }
