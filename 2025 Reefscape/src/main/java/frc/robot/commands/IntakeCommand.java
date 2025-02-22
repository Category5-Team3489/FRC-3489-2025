// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.enums.ElevatorState;
import frc.robot.enums.IndexState;
import frc.robot.enums.IntakeExtentionState;
import frc.robot.enums.IntakeRollerState;
import frc.robot.enums.OuttakeState;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Outtake;
import frc.robot.subsystems.intake.IntakeExtention;
import frc.robot.subsystems.intake.IntakeRoller;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/** An example command that uses an example subsystem. */
public class IntakeCommand extends SequentialCommandGroup {

    private static IntakeRoller intakeRoller = IntakeRoller.get();
    private static IntakeExtention intakeExtention = IntakeExtention.get();
    private static Index index = Index.get();
    private static Elevator elevator = Elevator.get();
    private static Outtake outtake = Outtake.get();

    public IntakeCommand() {

        addRequirements(intakeRoller, intakeExtention, index, elevator);

        addCommands(
                Commands.run(() -> {
                    outtake.updateSpeed(OuttakeState.Stop);
                    intakeExtention.updateCommand(IntakeExtentionState.IntakePosition);
                    intakeRoller.updateSpeed(IntakeRollerState.IntakeCollect, true);
                    elevator.updateCommand(ElevatorState.Down);
                }));
        if (intakeExtention.getSensor().equals(true)) { // && elevator.getBottomSensor()) {
            Commands.run(() -> {
                intakeExtention.updateCommand(IntakeExtentionState.HomePosition);
                index.updateSpeed(IndexState.Intake);
                intakeRoller.updateSpeed(IntakeRollerState.Stop, false);
                outtake.updateSpeed(OuttakeState.Intake);
                // Find the right time
                Commands.waitSeconds(2);
                outtake.updateSpeed(OuttakeState.Stop);
                elevator.updateCommand(ElevatorState.Up);
                outtake.updateSpeed(OuttakeState.Outtake);
            });
        }
        System.out.println("Intake update finished");
    }

    // addCommands(
    // Commands.runEnd(() -> {
    // intakeExtention.updateCommand(IntakeExtentionState.HomePosition);
    // intakeRoller.updateSpeed(IntakeRollerState.Intake);
    // elevator.updateCommand(ElevatorState.Down);
    // }, intakeExtention.getSensor().equals(true), intakeExtention));
    // System.out.println("Intake update finished");
    // }
}
