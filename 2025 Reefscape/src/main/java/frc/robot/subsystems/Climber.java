// package frc.robot.subsystems;

// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.spark.SparkMax;

// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants;
// import frc.robot.enums.ClimberState;

// public class Climber extends SubsystemBase {

// private static final Climber instance = new Climber();

// private SparkMax motor = new SparkMax(Constants.Elevator.RIGHT_MOTOR_ID,
// MotorType.kBrushless);

// public double speed = 0;

// public static Climber get() {
// return instance;
// }

// public Command setClimber(ClimberState climberState) {
// return Commands.runOnce(() -> {
// motor.set(climberState.getSpeed());
// speed = climberState.getSpeed();
// });
// }

// }
