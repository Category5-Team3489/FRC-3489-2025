// package frc.robot.subsystems.intake;

// import com.revrobotics.spark.SparkMax;

// import java.util.function.DoubleSupplier;

// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.spark.ClosedLoopSlot;
// import com.revrobotics.spark.SparkBase.ControlType;
// import com.revrobotics.spark.SparkClosedLoopController;
// import com.revrobotics.spark.SparkLowLevel.MotorType;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.wpilibj.DigitalInput;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants;
// import frc.robot.Robot;
// import frc.robot.enums.ElevatorState;
// import frc.robot.enums.IntakeExtentionState;

// public class IntakeExtention extends SubsystemBase {

// private static final IntakeExtention instance = new IntakeExtention();

// // TODO Update rev client values
// private final SparkMax motor = new
// SparkMax(Constants.IntakeActuator.MOTOR_ID, MotorType.kBrushless);
// private final SparkClosedLoopController pidController =
// motor.getClosedLoopController();

// private final RelativeEncoder encoder = motor.getEncoder();

// // TODO update to correct sensor type
// private final DigitalInput sensor = new
// DigitalInput(Constants.IntakeActuator.SENSOR_ID);

// private static final double CorrectionDegreesPerSecond = 5; // the speed

// // Gear ratio (motor rotates 30 times for one revolution of the actuator)
// private static final double gearRatio = 25;
// private final int sparkTicsPerRotation = 4096;

// // Intake is at its target angle if the error is within plus or minus this
// value
// private static final double AllowedErrorTics = 2.0;

// public double targetTics = IntakeExtentionState.MatchHome.getValue();

// private final IntakeRoller intakeRoller = IntakeRoller.get();

// public static IntakeExtention get() {
// return instance;
// }

// private double getEncoder() {
// return encoder.getPosition();
// }

// // Move the intake to the correct position
// private void setPosition() {
// if (targetTics == IntakeExtentionState.IntakePosition.getValue()) {
// pidController.setReference(targetTics, ControlType.kPosition,
// ClosedLoopSlot.kSlot1);
// } else {
// pidController.setReference(targetTics, ControlType.kPosition,
// ClosedLoopSlot.kSlot0);
// }
// }

// private void setTarget(double targetPosition) {
// targetTics = MathUtil.clamp(targetPosition,
// IntakeExtentionState.HomePosition.getValue(),
// IntakeExtentionState.IntakePosition.getValue());
// }

// public Command adjustManualHeight(double adjustPercent) {
// return Commands.run(() -> {
// double deltaDegrees = adjustPercent * CorrectionDegreesPerSecond *
// Robot.kDefaultPeriod;
// setTarget(targetTics + deltaDegrees);
// }, this);
// }

// public Command updateCommand(IntakeExtentionState intakeExtentionState) {
// return Commands.run(() -> {
// setTarget(intakeExtentionState.getValue());
// // System.out.println("Intake Update Command");
// }, this);
// }

// @Override
// public void periodic() {
// setPosition();
// checksensor();
// // System.out.println("Encoder: " + encoder.getPosition());

// }

// // Manual Testing
// public Command manualJoystick(double joystick) {
// return Commands.runOnce(() -> {
// motor.set(joystick * 0.3);
// });
// }

// public Command adjustManualAngle(double adjustPercent) {
// return Commands.run(() -> {
// double deltaDegrees = adjustPercent * CorrectionDegreesPerSecond *
// Robot.kDefaultPeriod;
// setTarget(targetTics + deltaDegrees);
// }, this);
// }

// private void checksensor() {
// // If the intake is extended: Check for sensor value
// if (targetTics == IntakeExtentionState.MatchHome.getValue()) {
// intakeRoller.checkSensor = false;
// // Else: Don't check sensor
// } else if (targetTics == IntakeExtentionState.IntakePosition.getValue()) {
// intakeRoller.checkSensor = true;
// }
// }

// }
