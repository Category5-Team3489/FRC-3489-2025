package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.enums.IntakeRollerState;

public class IntakeRoller extends SubsystemBase {

    private final SparkMax motor = new SparkMax(10, MotorType.kBrushless);

    private double speed = 0;

    public IntakeRoller() {

    }

    @Override
    public void periodic() {
        setIntake();
    }

    // Set the speed of the motor to the global intake variable
    private void setIntake() {
        motor.set(speed);
    }

    // Update the global intake speed variable based on the input enum
    public Command updateSpeed(IntakeRollerState state) {
        return Commands.runOnce(() -> speed = state.getSpeedPercent());
    }

}
