package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkMax;

import java.util.function.BooleanSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.enums.IntakeExtentionState;

public class IntakeExtention extends SubsystemBase {

    private static final IntakeExtention instance = new IntakeExtention();

    private final SparkMax motor = new SparkMax(Constants.IntakeActuator.MOTOR_ID, MotorType.kBrushless);
    private final SparkClosedLoopController pidController = motor.getClosedLoopController();

    // TODO update to correct sensor type
    private final DigitalInput sensor = new DigitalInput(Constants.IntakeActuator.SENSOR_ID);
    private final Trigger sensorTrigger = new Trigger(sensor::get);
    // the speed (10 degrees per second)
    private static final double CorrectionDegreesPerSecond = 10; // 50

    // Gear ratio (motor rotates 30 times for one revolution of the actuator)
    private static final double MotorRotationsPerRevolution = 30;

    // Convert revolutions to degrees (Used in PID)(amount of rotations motor has to
    // make for the actuator to move one degree)
    private static final double MotorRotationsPerDegree = MotorRotationsPerRevolution / 360.0;
    private static final double DegreesPerMotorRotation = 1.0 / MotorRotationsPerDegree;

    // Intake is at its target angle if the error is within plus or minus this value
    private static final double AllowedErrorDegrees = 2.0;

    private double targetDegrees = 0;

    public static IntakeExtention get() {
        return instance;
    }

    // Move the intake to the correct position
    private void setPosition(double target) {
        setTarget(target);
        double targetRotations = target * MotorRotationsPerDegree;
        pidController.setReference(targetRotations, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }

    private void setTarget(double targetPosition) {
        targetDegrees = MathUtil.clamp(targetPosition,
                IntakeExtentionState.HomePosition.getValue(), IntakeExtentionState.IntakePosition.getValue());
    }

    public Command adjustManualHeight(double adjustPercent) {
        return Commands.run(() -> {
            double deltaDegrees = adjustPercent * CorrectionDegreesPerSecond * Robot.kDefaultPeriod;
            setTarget(targetDegrees + deltaDegrees);
        }, this);
    }

    public Command updateCommand(IntakeExtentionState intakeExtentionState) {
        return Commands.run(() -> {
            setTarget(intakeExtentionState.getValue());
            System.out.println("Intake Update Command");
        }, this);
    }

    @Override
    public void periodic() {
        setPosition(targetDegrees);
    }

    public BooleanSupplier getSensor() {
        return sensorTrigger;
    }
}
