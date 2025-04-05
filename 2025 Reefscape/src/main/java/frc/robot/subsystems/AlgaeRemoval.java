package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.enums.AlgaeRemovalState;

// import frc.robot.subsystems.intake.IntakeExtention;

public class AlgaeRemoval extends SubsystemBase {

    private static final AlgaeRemoval instance = new AlgaeRemoval();

    // TODO Neo 550
    private final SparkMax motor = new SparkMax(Constants.AlgaeRemoval.MOTOR_ID, MotorType.kBrushless);

    public double speed = 0;

    public static AlgaeRemoval get() {
        return instance;
    }

    // Update the global speed variables based on the input enums
    public Command updateSpeed(AlgaeRemovalState state) {
        return Commands.runOnce(() -> {
            motor.set(speed);
            speed = state.getSpeedPercent();
        });
    }

}
