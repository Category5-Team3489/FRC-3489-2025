package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.enums.IndexState;
import frc.robot.enums.OuttakeState;
// import frc.robot.subsystems.intake.IntakeExtention;

public class Index extends SubsystemBase {

    private static final Index instance = new Index();

    // Neo 550
    private final SparkMax leftMotor = new SparkMax(Constants.Index.LEFT_MOTOR_ID, MotorType.kBrushless);
    // private final SparkMax rightMotor = new
    // SparkMax(Constants.Index.RIGHT_MOTOR_ID, MotorType.kBrushless);

    // private double rightSpeed = 0;
    public double speed = 0;

    public static Index get() {
        return instance;
    }

    @Override
    public void periodic() {
        // setIndex();
    }

    // Set the speed of the motor to the global speed variables
    private void setIndex() {
        leftMotor.set(speed);
        // rightMotor.set(rightSpeed);

    }

    // Update the global speed variables based on the input enums
    public Command updateSpeed(IndexState state) {
        return Commands.runOnce(() -> {
            speed = state.getSpeedPercent();

            leftMotor.set(speed);
        });
    }

}
