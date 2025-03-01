// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

// import frc.robot.commands.IntakeCommand;
import frc.robot.enums.ElevatorState;
import frc.robot.enums.IndexState;
import frc.robot.enums.IntakeRollerState;
// import frc.robot.enums.IntakeExtentionState;
// import frc.robot.enums.IntakeRollerState;
import frc.robot.enums.OuttakeState;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Outtake;
import frc.robot.subsystems.intake.IntakeExtention;
import frc.robot.subsystems.intake.IntakeRoller;

public class RobotContainer {
        private final Outtake outtake = Outtake.get();
        private final Elevator elevator = Elevator.get();
        // private final IntakeCommand intakeCommand = new IntakeCommand();
        // private final IntakeExtention intake = new IntakeExtention();
        // private final IntakeRoller intakeRoller = new IntakeRoller();
        private final Index index = Index.get();
        private final IntakeExtention intakeExtention = IntakeExtention.get();
        private final IntakeRoller intakeRoller = IntakeRoller.get();

        private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top
                                                                                      // speed
        private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per
                                                                                          // second
                                                                                          // max angular velocity

        /* Setting up bindings for necessary control of the swerve drive platform */
        private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive
                                                                                 // motors
        private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
        private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
        private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        private final Telemetry logger = new Telemetry(MaxSpeed);

        private final CommandXboxController driverController = new CommandXboxController(0);
        private final CommandXboxController manipulatorController = new CommandXboxController(1);

        public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

        /* Path follower */
        private final SendableChooser<Command> autoChooser;

        public RobotContainer() {
                autoChooser = AutoBuilder.buildAutoChooser("New Auto");

                SmartDashboard.putData("Auto Mode", autoChooser);

                NamedCommands.registerCommand("Outtake", outtake.updateSpeed(OuttakeState.Outtake));
                // NamedCommands.registerCommand("Intake", intakeCommand);
                NamedCommands.registerCommand("L1",
                                elevator.updateCommand(ElevatorState.L1));
                NamedCommands.registerCommand("L2",
                                elevator.updateCommand(ElevatorState.L2));
                NamedCommands.registerCommand("L3",
                                elevator.updateCommand(ElevatorState.L3));
                NamedCommands.registerCommand("Home",
                                elevator.updateCommand(ElevatorState.Down));

                NamedCommands.registerCommand("TEST", Commands.run(() -> {
                        System.out.println("TESTING------------------");
                }));

                NamedCommands.registerCommand("tests", Commands.run(() -> {
                        System.out.println("!!!!!!!!!!!!!!!!!!!!!!!!!");
                }));

                configureBindings();
        }

        private void configureBindings() {
                // manipulatorController.a().onTrue(intakeRoller.updateSpeed(IntakeRollerState.IntakeCollect,
                // true));

                // // *************** INTAKE RIB ROLLER ***********************
                // manipulatorController.start().onTrue(intakeRoller.updateSpeed(IntakeRollerState.IntakeCollect,
                // true));

                // manipulatorController.y().onTrue(intakeRoller.updateSpeed(IntakeRollerState.IntakeCollect,
                // true));

                // manipulatorController.a().onTrue(intakeRoller.updateSpeed(IntakeRollerState.Stop,
                // true));

                // manipulatorController.b().onTrue(Commands.runOnce(() ->
                // intakeRoller.testing()));

                // manipulatorController.b().onTrue(canRangeTesting.updateSpeed(-0.15, true));

                // manipulatorController.a().onTrue(canRangeTesting.updateSpeed(-0.15, false));
                // // ***************************************************************
                // manipulatorController.a().onTrue(intakeExtention.manualJoystick(1));

                // manipulatorController.b().onTrue(intakeExtention.manualJoystick(-1));

                // manipulatorController.rightBumper().onTrue(
                // intakeExtention.updateCommand(IntakeExtentionState.HomePosition));

                // manipulatorController.leftBumper().onTrue(
                // intakeExtention.updateCommand(IntakeExtentionState.IntakePosition));

                // // ***************************************************************

                manipulatorController.rightBumper().onTrue(outtake.updateSpeed(OuttakeState.Outtake));

                manipulatorController.back().onTrue(intakeRoller.updateSpeed(IntakeRollerState.IntakeCollect, true));

                // manipulatorController.rightBumper().onTrue(outtake.updateSpeed(OuttakeState.Stop));

                manipulatorController.leftBumper().onTrue(index.updateSpeed(IndexState.Stop));

                manipulatorController.start().onTrue(index.updateSpeed(IndexState.Intake));

                // //!!!!!!!!!!!!!!!!!!!!!!!!!!!! ELEVATOR !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                // manipulatorController.y().onTrue(elevator.manualJoystick(0));

                // manipulatorController.axisLessThan(5,
                // -0.1).whileTrue(elevator.manualJoystick(1));
                // manipulatorController.axisGreaterThan(5,
                // 0.1).whileTrue(elevator.manualJoystick(-1));

                // TODO Uncomment
                manipulatorController.a().onTrue(elevator.updateCommand(ElevatorState.Down));

                manipulatorController.x().onTrue(elevator.updateCommand(ElevatorState.L1));

                manipulatorController.b().onTrue(elevator.updateCommand(ElevatorState.L2));

                manipulatorController.y().onTrue(elevator.updateCommand(ElevatorState.L3));

                // //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

                // -----------------BINDING METHODS-----------------------------------------
                getDrivetrainBindings();
                // getElevatorBindings();
                // getOuttakeBindings(); // RUN = LB; STOP = RB
                // getIntakeBindings();
                // getIndexBindings(); // RUN = START; STOP = BACK

        }

        private void getDrivetrainBindings() {
                // Note that X is defined as forward according to WPILib convention,
                // and Y is defined as to the left according to WPILib convention.
                drivetrain.setDefaultCommand(
                                // Drivetrain will execute this command periodically
                                drivetrain.applyRequest(() -> drive
                                                .withVelocityX(-driverController.getLeftY() * MaxSpeed) // Drive
                                                                                                        // forward
                                                                                                        // with
                                                // negative Y
                                                // (forward)
                                                .withVelocityY(-driverController.getLeftX() * MaxSpeed) // Drive left
                                                                                                        // with negative
                                                                                                        // X (left)
                                                .withRotationalRate(-driverController.getRightX() * MaxAngularRate) // Drive
                                                                                                                    // counterclockwise
                                                                                                                    // with
                                // negative X (left)
                                ));

                // driverController.a().whileTrue(drivetrain.applyRequest(() -> brake));
                // driverController.b().whileTrue(drivetrain.applyRequest(
                // () -> point.withModuleDirection(
                // new Rotation2d(-driverController.getLeftY(),
                // -driverController.getLeftX()))));

                // driverController.pov(0)
                // .whileTrue(drivetrain.applyRequest(
                // () -> forwardStraight.withVelocityX(0.5).withVelocityY(0)));
                // driverController.pov(180)
                // .whileTrue(drivetrain.applyRequest(
                // () -> forwardStraight.withVelocityX(-0.5).withVelocityY(0)));

                // Run SysId routines when holding back/start and X/Y.
                // Note that each routine should be run exactly once in a single log.
                // driverController.back().and(driverController.y())
                // .whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
                // driverController.back().and(driverController.x())
                // .whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
                // driverController.start().and(driverController.y())
                // .whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
                // driverController.start().and(driverController.x())
                // .whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

                // reset the field-centric heading on left bumper press
                driverController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

                drivetrain.registerTelemetry(logger::telemeterize);

        }

        private void getElevatorBindings() {
                // manipulatorController.a().onTrue(elevator.updateSpeed()); for manual use
                // manipulatorController.axisLessThan(5,
                // -0.1).whileTrue(elevator.adjustManualHeight(1)); for manual use

                manipulatorController.y().onTrue(elevator.updateCommand(ElevatorState.L1));
                manipulatorController.b().onTrue(elevator.updateCommand(ElevatorState.L2));
                manipulatorController.a().onTrue(elevator.updateCommand(ElevatorState.L3));

        }

        private void getOuttakeBindings() {
                manipulatorController.leftBumper().onTrue(outtake.updateSpeed(OuttakeState.Intake));
                manipulatorController.rightBumper().onTrue(outtake.updateSpeed(OuttakeState.Stop));
        }

        private void getIntakeBindings() {
                // manipulatorController.start()
                // .onTrue(intakeRoller.updateSpeed(IntakeRollerState.IntakeCollect, true));
                // manipulatorController.back().onTrue(intakeRoller.updateSpeed(IntakeRollerState.Stop,
                // false));
        }

        private void getIndexBindings() {
                manipulatorController.start().onTrue(index.updateSpeed(IndexState.Intake));
                manipulatorController.back().onTrue(index.updateSpeed(IndexState.Stop));
        }

        public Command getAutonomousCommand() {
                /* Run the path selected from the auto chooser */
                return new PathPlannerAuto("(3) Far Left to 3B");
        }
}
