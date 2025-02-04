package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.enums.ElevatorState;

public class Elevator extends SubsystemBase {

    // TODO Add the right motor as a follower for left motor in Rev client
    // TODO Add the pid values for the left motor in the rev client
    private final SparkMax rightMotor = new SparkMax(12, MotorType.kBrushless);
    private final SparkMax leftMotor = new SparkMax(13, MotorType.kBrushless);

    private final DigitalInput topSwitch = new DigitalInput(4);
    private final DigitalInput bottomSwitch = new DigitalInput(0);

    private static final double CorrectionInchesPerSecond = 5;

    // TODO Update this when we have a gear ratio
    // to make the motor rotate once (gear-ratio)
    private static final double MotorRotationsPerRevolution = 10;

    // the amount of times motor should spin to make it move one inch
    private static final double MotorRotationsPerInch = MotorRotationsPerRevolution / 360.0;
    // the amount of inches it should move to make the motor rotate once
    private static final double InchesPerMotorRotation = 1.0 / MotorRotationsPerInch;

    private static final double AllowedErrorInches = 2.0;

    private final SparkClosedLoopController pidControllerLeft = leftMotor.getClosedLoopController();

    private final RelativeEncoder encoder = leftMotor.getEncoder();

    // TODO do the math correctly after getting the actual numbers
    private double currentHeight = encoder.getPosition() * 10;

    private double targetInches = 0;

    @Override
    public void periodic() {
        // setHeight(targetInches);
        checkLimits(targetInches);
    }

    // Move the elevator to the correct height
    private void setHeight(double positionHeight) {
        setTargetInches(positionHeight);
        double targetRotations = positionHeight * MotorRotationsPerInch;
        pidControllerLeft.setReference(targetRotations, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }

    // If the limit switches are hit: dont continue moving in that direction; If
    // switches not hit: run setHeight();
    private void checkLimits(double target) {
        // If the top limit switch is hit
        if (topSwitch.get()) {
            // If you are trying to go down: drive normally
            if (targetInches < currentHeight) {
                setHeight(target);
                System.out.println("If you are trying to go down: drive normally");
            }
            // If you are trying to go up more: Stop Motor
            else if (targetInches > currentHeight) {
                leftMotor.stopMotor();
            }
        }
        // if the bottom switch is on
        else if (bottomSwitch.get()) {
            // if you want to go up: move normally
            if (targetInches >= currentHeight) {
                setHeight(target);
            }
            // if you want to go more down: stop motor
            else {
                rightMotor.stopMotor();
            }
        }

        else {
            setHeight(target);
            setHeight(target);
            System.out.println("RUN!!!!!!!!!!!!!!!!!!!!!!!111");
        }
    }

    private void setTargetInches(double positionHeight) {
        targetInches = MathUtil.clamp(positionHeight,
                ElevatorState.Down.getHeigtInches(), ElevatorState.Up.getHeigtInches());
    }

    public Command adjustManualHeight(double adjustPercent) {
        return Commands.run(() -> {
            double deltaDegrees = adjustPercent * CorrectionInchesPerSecond * Robot.kDefaultPeriod;
            setTargetInches(targetInches + deltaDegrees);
        }, this);
    }

    public Command updateCommand(DoubleSupplier heightInchesSupplier) {
        return Commands.run(() -> {
            setTargetInches(heightInchesSupplier.getAsDouble());
        }, this);
    }
}
