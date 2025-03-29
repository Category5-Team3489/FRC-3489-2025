// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.autoPlacement.Align;
import frc.robot.enums.ClimberState;
import frc.robot.enums.ElevatorState;
import frc.robot.enums.IndexState;
import frc.robot.enums.OuttakeState;
import frc.robot.enums.SpeedLimitState;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.ElevatorLimelight;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Outtake;

public class RobotContainer {
    private final Outtake outtake = Outtake.get();
    private final Elevator elevator = Elevator.get();
    // private final IntakeCommand intakeCommand = new IntakeCommand();
    private final Index index = Index.get();
    private final Climber climber = Climber.get();
    private final ElevatorLimelight limelight = ElevatorLimelight.get();
    // private final IntakeExtention intakeExtention = IntakeExtention.get();
    // private final IntakeRoller intakeRoller = IntakeRoller.get();
    // private final ElevatorLimelight limelight = ElevatorLimelight.get();

    private boolean alignScheduled = false;

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

    private final SwerveRequest.RobotCentric driveRobotRel = new SwerveRequest.RobotCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandXboxController manipulatorController = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private final Command align = new Align(drivetrain).onlyWhile(() -> isNotDriving())
            .withInterruptBehavior(InterruptionBehavior.kCancelSelf);

    // public final ElevatorLimelight limelight = ElevatorLimelight.get();

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        autoChooser = AutoBuilder.buildAutoChooser("New Auto");

        SmartDashboard.putData("Auto Mode", autoChooser);

        NamedCommands.registerCommand("Outtake", outtake.updateSpeed(OuttakeState.Outtake));
        NamedCommands.registerCommand("Index", index.updateSpeed(IndexState.Outtake));
        // NamedCommands.registerCommand("Intake", intakeCommand);
        NamedCommands.registerCommand("L1", elevator.updateCommand(ElevatorState.L1));
        NamedCommands.registerCommand("L2", elevator.updateCommand(ElevatorState.L2));
        NamedCommands.registerCommand("L3", elevator.updateCommand(ElevatorState.L3));
        NamedCommands.registerCommand("Home Elevator", elevator.updateCommand(ElevatorState.Down));

        NamedCommands.registerCommand("TEST", Commands.run(() -> {
            System.out.println("TESTING------------------");
        }));

        NamedCommands.registerCommand("tests", Commands.run(() -> {
            System.out.println("!!!!!!!!!!!!!!!!!!!!!!!!!");
        }));

        configureBindings();
    }

    private void configureBindings() {

        // driverController.rightBumper().onTrue(Commands.runOnce( () ->
        // align.schedule()));

        driverController.start().onTrue(Commands.run(() -> {
            // if (alignScheduled) {
            // align.cancel();
            // System.out.println("Cancle!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
            // alignScheduled = false;
            // } else {
            align.schedule();
            alignScheduled = true;
            System.out.println("++++++++++++++++++++++++++++++++++++++++++++");

        }));

        // manipulatorController.axisLessThan(5, -0.1).whileTrue(
        // intakeExtention.adjustManualAngle(-1));

        // manipulatorController.axisGreaterThan(5, 0.1).whileTrue(
        // intakeExtention.adjustManualAngle(1));

        manipulatorController.axisLessThan(2, -0.1).whileTrue(
                elevator.adjustManualAngle(1));

        manipulatorController.axisGreaterThan(2, 0.1).whileTrue(
                elevator.adjustManualAngle(-1));

        // // ================= INTAKE ROLLER ===================================

        // manipulatorController.povUp().onTrue(intakeRoller.setSpeedCommand(IntakeRollerState.IntakeCollect));

        // ======================================================================
        // -----------------BINDING METHODS-----------------------------------------
        getDrivetrainBindings();
        getElevatorBindings();
        getOuttakeBindings(); // RUN = LB; STOP = RB
        getClimberBindings();

        // // getIntakeExtendBindings();
        // // getIntakeRollerBindings();
        getIndexBindings(); // RUN = START; STOP = BACK

    }

    private boolean isNotDriving() {
        return Math.abs(driverController.getLeftX()) < 0.1 &&
                Math.abs(driverController.getLeftY()) < 0.1
                && Math.abs(driverController.getRightX()) < 0.1;
    }

    private void getDrivetrainBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() -> drive
                        .withVelocityX(-driverController.getLeftY() * MaxSpeed * drivetrain.getSpeedLimit()) // Drive
                        // forward
                        // with
                        // negative Y
                        // (forward)
                        .withVelocityY(-driverController.getLeftX() * MaxSpeed * drivetrain.getSpeedLimit()) // Drive
                                                                                                             // left
                        // with negative
                        // X (left)
                        .withRotationalRate(-driverController.getRightX() * MaxAngularRate * drivetrain.getSpeedLimit()) // Drive
                // counterclockwise
                // with
                // negative X (left)
                ));

        // final SwerveRequest.FieldCentricFacingAngle driveFacingAngle = new
        // SwerveRequest.FieldCentricFacingAngle()
        // .withDeadband(Constants.Drivetrain.MaxMetersPerSecond * 0.1)
        // .withRotationalDeadband(Constants.Drivetrain.MaxRadiansPerSecond * 0.1) //
        // Add a
        // // 10%
        // // deadband
        // .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want
        // field-centric
        // driving in open loop

        // driverController.y().whileTrue(
        // drivetrain.applyRequest(() -> driveFacingAngle
        // .withVelocityX(-driverController.getLeftY() *
        // Constants.Drivetrain.MaxMetersPerSecond)
        // .withVelocityY(-driverController.getLeftX() *
        // Constants.Drivetrain.MaxMetersPerSecond)
        // .withTargetDirection(Rotation2d.fromDegrees(0))));
        // driverController.b().whileTrue(
        // drivetrain.applyRequest(() -> driveFacingAngle
        // .withVelocityX(-driverController.getLeftY() *
        // Constants.Drivetrain.MaxMetersPerSecond)

        // .withVelocityY(-driverController.getLeftX() *
        // Constants.Drivetrain.MaxMetersPerSecond)
        // .withTargetDirection(Rotation2d.fromDegrees(270))));
        // driverController.a().whileTrue(
        // drivetrain.applyRequest(() -> driveFacingAngle
        // .withVelocityX(-driverController.getLeftY() *
        // Constants.Drivetrain.MaxMetersPerSecond)

        // .withVelocityY(-driverController.getLeftX() *
        // Constants.Drivetrain.MaxMetersPerSecond)
        // .withTargetDirection(Rotation2d.fromDegrees(180))));
        // driverController.x().whileTrue(
        // drivetrain.applyRequest(() -> driveFacingAngle
        // .withVelocityX(-driverController.getLeftY() *
        // Constants.Drivetrain.MaxMetersPerSecond)

        // .withVelocityY(-driverController.getLeftX() *
        // Constants.Drivetrain.MaxMetersPerSecond)
        // .withTargetDirection(Rotation2d.fromDegrees(90))));

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

        // TODO !!!Uncomment this for Limelight Testing!!!
        // driverController.rightBumper().onTrue(Commands.runOnce(() -> {
        // alignToReefTagRelative.schedule();
        // }));

        drivetrain.registerTelemetry(logger::telemeterize);
        // DriverStation.silenceJoystickConnectionWarning(true);

        driverController.leftBumper().onTrue(Commands.runOnce(() -> drivetrain.setSpeedLimit(SpeedLimitState.Full)));
        driverController.rightBumper()
                .onTrue(Commands.runOnce(() -> drivetrain.setSpeedLimit(SpeedLimitState.Forth)));
        // Reset the speed when button is released
        driverController.leftBumper().onFalse(Commands.runOnce(() -> drivetrain.setSpeedLimit(SpeedLimitState.Half)));
        driverController.rightBumper()
                .onFalse(Commands.runOnce(() -> drivetrain.setSpeedLimit(SpeedLimitState.Half)));
    }

    private void getElevatorBindings() {

        manipulatorController.y().onTrue(elevator.updateCommand(ElevatorState.L3));
        manipulatorController.b().onTrue(elevator.updateCommand(ElevatorState.L2));
        manipulatorController.x().onTrue(elevator.updateCommand(ElevatorState.L1));
        manipulatorController.a().onTrue(elevator.updateCommand(ElevatorState.Down));
        manipulatorController.rightBumper().onTrue(elevator.updateCommand(ElevatorState.HP));

    }

    private void getOuttakeBindings() {
        manipulatorController.back().onTrue(Commands.runOnce(() -> {
            if (outtake.speed == OuttakeState.Stop.getSpeedPercent()) {
                outtake.updateSpeed(OuttakeState.Outtake).schedule();
            } else {
                outtake.updateSpeed(OuttakeState.Stop).schedule();
            }
        }));

    }

    private void getClimberBindings() {
        manipulatorController.povUp().onTrue(Commands.runOnce(() -> {
            if (climber.speed == ClimberState.Off.getSpeed()) {
                climber.setClimber(ClimberState.On).schedule();
            } else {
                climber.setClimber(ClimberState.Off).schedule();
            }
        }));
    }

    private void getIntakeExtendBindings() {
        // final IntakeCommandOne intakeCommandOne = new IntakeCommandOne();
        // final IntakeCommandTwo intakeCommandThree = new IntakeCommandTwo();
        // final IntakeCommandThree intakeCommandTwo = new IntakeCommandThree();

        // manipulatorController.leftBumper().onTrue(Commands.runOnce(() -> {
        // if (intakeExtention.targetTics ==
        // IntakeExtentionState.IntakePosition.getValue()) {
        // intakeExtention.updateCommand(IntakeExtentionState.MatchHome).schedule();
        // } else {
        // intakeCommandOne.schedule();
        // }
        // }));

        // manipulatorController.rightBumper().onTrue(Commands.runOnce(() ->
        // intakeCommandTwo.schedule()));
        // manipulatorController.leftTrigger().onTrue(Commands.runOnce(() ->
        // intakeCommandThree.schedule()));

    }

    // private void getIntakeRollerBindings() {
    // manipulatorController.povUp().onTrue(Commands.runOnce(() -> {
    // if (intakeRoller.speed == IntakeRollerState.Stop.getSpeedPercent()) {
    // intakeRoller.setSpeedCommand(IntakeRollerState.Test).schedule();
    // } else {
    // intakeRoller.setSpeedCommand(IntakeRollerState.Stop).schedule();
    // }
    // }));

    // manipulatorController.povDown().onTrue(Commands.runOnce(() -> {
    // if (intakeRoller.speed == IntakeRollerState.Stop.getSpeedPercent()) {
    // intakeRoller.setSpeedCommand(IntakeRollerState.IntakeTransfer).schedule();
    // } else {
    // intakeRoller.setSpeedCommand(IntakeRollerState.Stop).schedule();
    // }
    // }));

    // manipulatorController.povRight().onTrue(Commands.runOnce(() -> {
    // if (intakeRoller.speed == IntakeRollerState.Stop.getSpeedPercent()) {
    // intakeRoller.setSpeedCommand(IntakeRollerState.Outtake).schedule();
    // } else {
    // intakeRoller.setSpeedCommand(IntakeRollerState.Stop).schedule();
    // }
    // }));

    // final IntakeCommandTwo intakeCommandTwo = new IntakeCommandTwo();

    // manipulatorController.povLeft().onTrue(Commands.runOnce(() ->
    // intakeCommandTwo.schedule()));
    // }

    private void getIndexBindings() {
        manipulatorController.start().onTrue(Commands.runOnce(() -> {
            if (index.speed == IndexState.Stop.getSpeedPercent()
                    || index.speed == IndexState.Intake.getSpeedPercent()) {
                index.updateSpeed(IndexState.Outtake).schedule();
            } else {
                index.updateSpeed(IndexState.Stop).schedule();
            }
        }));

        manipulatorController.leftBumper().onTrue(Commands.runOnce(() -> {
            if (index.speed == IndexState.Stop.getSpeedPercent()
                    || index.speed == IndexState.Outtake.getSpeedPercent()) {
                index.updateSpeed(IndexState.Intake).schedule();
            } else {
                index.updateSpeed(IndexState.Stop).schedule();
            }
        }));
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return new PathPlannerAuto("Leave");
    }
}
