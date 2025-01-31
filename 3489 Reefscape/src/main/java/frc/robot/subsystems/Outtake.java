package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.enums.OuttakeState;

public class Outtake extends SubsystemBase {

    private final SparkMax motor = new SparkMax(14, MotorType.kBrushless);

    private double speed = 0;

    public Outtake() {

    }

    @Override
    public void periodic() {
        setOuttake();
    }

    // Set the speed of the motor to the global outtake variable
    private void setOuttake() {
        motor.set(speed);
    }

    // Update the global outtake speed variable based on the input enum
    public Command updateSpeed(OuttakeState state) {
        return Commands.runOnce(() -> speed = state.getSpeedPercent());
    }

}
