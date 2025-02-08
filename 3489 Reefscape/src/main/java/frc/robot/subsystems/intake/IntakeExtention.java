package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.enums.ElevatorState;
import frc.robot.enums.IntakeExtentionState;

public class IntakeExtention extends SubsystemBase {

    private final SparkMax motor = new SparkMax(Constants.IntakeActuator.MOTOR_ID, MotorType.kBrushless);

    // TODO update to correct sensor type
    private final DigitalInput sensor = new DigitalInput(Constants.IntakeActuator.SENSOR_ID);

    private static final double CorrectionInchesPerSecond = 5;
    // TODO Update this when we have a gear ratio
    // to make the motor rotate once (gear-ratio)
    private static final double MotorRotationsPerRevolution = 10;
    // the amount of times motor should spin to make it move one inch
    private static final double MotorRotationsPerUnit = MotorRotationsPerRevolution / 360.0;
    // the amount of inches it should move to make the motor rotate once
    private static final double InchesPerMotorRotation = 1.0 / MotorRotationsPerUnit;
    private static final double AllowedErrorInches = 2.0;

    private final SparkClosedLoopController pidController = motor.getClosedLoopController();
    private final RelativeEncoder encoder = motor.getEncoder();

    // TODO do the math correctly after getting the actual numbers
    private double currentHeight = encoder.getPosition() * 10;

    private double targetInches = 0;

    @Override
    public void periodic() {
        setPosition(targetInches);
    }

    // Move the intake to the correct position
    private void setPosition(double target) {
        setTarget(target);
        double targetRotations = target * MotorRotationsPerUnit;
        pidController.setReference(targetRotations, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }

    private void setTarget(double targetPosition) {
        targetInches = MathUtil.clamp(targetPosition,
                IntakeExtentionState.HomePosition.getValue(), IntakeExtentionState.IntakePosition.getValue());
    }

    public Command adjustManualHeight(double adjustPercent) {
        return Commands.run(() -> {
            double deltaDegrees = adjustPercent * CorrectionInchesPerSecond * Robot.kDefaultPeriod;
            setTarget(targetInches + deltaDegrees);
        }, this);
    }

    public Command updateCommand(IntakeExtentionState elevatorState) {
        return Commands.run(() -> {
            setTarget(elevatorState.getValue());
        }, this);
    }
}
