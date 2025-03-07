// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {

  // private final VortexTesting vortexTesting = VortexTesting.get();
  private final CANrangeTesting canRangeTesting = CANrangeTesting.get();

  private final CommandXboxController driverController = new CommandXboxController(0);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    // driverController.a().onTrue(vortexTesting.runMotor(-0.15));
    // driverController.b().onTrue(vortexTesting.runMotor(-0.70));
    // driverController.x().onTrue(vortexTesting.runMotor(0));

    // canRangeTesting.setDefaultCommand(Commands.run(() ->
    // canRangeTesting.setIntake()));

    driverController.y().onTrue(canRangeTesting.updateSpeed(-0.15, false));

    driverController.b().onTrue(canRangeTesting.updateSpeed(-0.15, true));

    driverController.a().onTrue(canRangeTesting.updateSpeed(-0.15, true));

  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
