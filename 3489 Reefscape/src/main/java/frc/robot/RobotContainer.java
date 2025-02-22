// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.IntakeCommand;
import frc.robot.enums.ElevatorState;
import frc.robot.enums.OuttakeState;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Outtake;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
	// The robot's subsystems and commands are defined here...
	private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
	private final Outtake outtake = Outtake.get();
	private final Elevator elevator = Elevator.get();
	private final IntakeCommand intakeCommand = new IntakeCommand();

	// Replace with CommandPS4Controller or CommandJoystick if needed
	private final CommandXboxController m_driverController = new CommandXboxController(
			OperatorConstants.kDriverControllerPort);

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {

		NamedCommands.registerCommand("Outtake", outtake.updateSpeed(OuttakeState.Outtake));
		NamedCommands.registerCommand("Intake", intakeCommand);
		NamedCommands.registerCommand("L1", elevator.updateCommand(ElevatorState.L1));
		NamedCommands.registerCommand("L2", elevator.updateCommand(ElevatorState.L2));
		NamedCommands.registerCommand("L3", elevator.updateCommand(ElevatorState.L3));
		// Configure the trigger bindings
		configureBindings();
	}

	private void configureBindings() {

		// Code Testing Buttons
		m_driverController.a().onTrue(elevator.updateCommand(ElevatorState.L3));
		m_driverController.b().onTrue(outtake.updateSpeed(OuttakeState.Intake));

		m_driverController.x().onTrue(intakeCommand);

	}

	public Command getAutonomousCommand() {
		// An example command will be run in autonomous
		return Autos.exampleAuto(m_exampleSubsystem);
	}
}
