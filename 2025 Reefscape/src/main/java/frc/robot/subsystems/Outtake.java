package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.enums.OuttakeState;

public class Outtake extends SubsystemBase {

    private static final Outtake instance = new Outtake();

    // Neo 550
    private final SparkMax motor = new SparkMax(Constants.Outtake.MOTOR_ID, MotorType.kBrushless);
    private final DigitalInput sensor = new DigitalInput(Constants.Outtake.SENSOR_ID);

    public double speed = 0;

    public static Outtake get() {
        return instance;
    }

    @Override
    public void periodic() {
        // setOuttake();
    }

    // Set the speed of the motor to the global outtake variable
    private void setOuttake() {
        motor.set(speed);
    }

    // Update the global outtake speed variable based on the input enum
    public Command updateSpeed(OuttakeState state) {
        // return Commands.runOnce(() -> speed = state.getSpeedPercent());
        return Commands.runOnce(() -> {
            System.out.println("UPDATE SPEED");
            motor.set(state.getSpeedPercent());
            speed = state.getSpeedPercent();
        });

    }

}
