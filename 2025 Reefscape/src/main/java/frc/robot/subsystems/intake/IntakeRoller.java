// package frc.robot.subsystems.intake;

// import com.ctre.phoenix6.configs.CANrangeConfiguration;
// import com.ctre.phoenix6.hardware.CANrange;
// import com.revrobotics.spark.SparkFlex;
// import com.revrobotics.spark.SparkLowLevel.MotorType;

// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants;
// import frc.robot.enums.IntakeRollerState;

// public class IntakeRoller extends SubsystemBase {
// private static final IntakeRoller instance = new IntakeRoller();

// // Devices (Vortex & CANrange sensor)
// private final SparkFlex motor = new
// SparkFlex(Constants.IntakeRoller.MOTOR_ID, MotorType.kBrushless);
// private final CANrange CANrange = new
// CANrange(Constants.IntakeRoller.SENSOR_ID,
// Constants.IntakeRoller.CAN_RANGE_CAN_BUS);

// CANrangeConfiguration configs = new CANrangeConfiguration(); // Configure the
// CANrange for basic use

// public double speed = 0; // Target Speed of Intake
// public boolean checkSensor = true; // Should the motor stop if the Sensor is
// triggered?

// public static IntakeRoller get() {
// return instance;
// }

// public IntakeRoller() {
// CANrange.getConfigurator().apply(configs); // Write these configs to the
// CANrange
// }

// public void testing() {
// motor.set(IntakeRollerState.IntakeTransfer.getSpeedPercent());
// }

// public double returnRange() {
// var distance = CANrange.getDistance(); //Get Distance
// var currentDistance = distance.refresh(); //Refresh the value

// // converts the current distance to a double
// double distanceDouble = currentDistance.getValueAsDouble();
// return distanceDouble;
// }

// @Override
// public void periodic() {
// // setIntake();
// // System.out.println("---------------------------" + checkSensor);
// checkSensor();
// // hasTimeElapsed();

// // TODO TEST THIS
// //
// System.out.println("----------------------------------------------------SENSOR:"
// // + CANrange.getIsDetected());

// // System.out.println("________________________________" + returnRange());

// }

// // Set the speed of the motor to the global intake variable
// private void setIntake() {
// motor.set(speed);
// }

// // TODO Add shuffleboard logic to stop getting sensor values if sensor
// damaged
// // If current command requires sensor: stop motor when value is in range
// private void checkSensor() {
// if (checkSensor) {
// double sensorValue = returnRange();
// // System.out.println("___________________" + CANrange.getDistance());
// if (sensorValue <= Constants.IntakeRoller.SENSOR_RANGE) {
// motor.set(IntakeRollerState.Stop.getSpeedPercent());
// // System.out.println("STOP____________________________________________");
// }
// } else {
// return;
// }
// }

// // public void stopMotor() {
// // if (hasTime) {
// // motor.set(IntakeRollerState.Stop.getSpeedPercent());
// // }
// // }

// // public boolean hasTimeElapsed() {
// // if (timer.hasElapsed(0.01)) {
// // hasTime = true;
// // } else {
// // hasTime = false;
// // }
// // stopMotor();
// // return hasTime;
// // }

// // // Update the global intake speed variable based on the input enum
// // public Command updateSpeed(IntakeRollerState state, boolean checkSensor) {
// // // this.checkSensor = checkSensor;
// // return Commands.runOnce(() -> speed = state.getSpeedPercent());
// // }

// public Command setSpeedCommand(IntakeRollerState intakeRollerState) {
// return Commands.runOnce(() -> {
// motor.set(intakeRollerState.getSpeedPercent());
// speed = intakeRollerState.getSpeedPercent();
// }, this);
// }

// }
