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
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.enums.ElevatorState;

public class Elevator extends SubsystemBase {

    private static final Elevator instance = new Elevator();

    // TODO Add the right motor as a follower for left motor in Rev client
    // TODO Add the pid values for the left motor in the rev client
    private final SparkMax rightMotor = new SparkMax(Constants.Elevator.RIGHT_MOTOR_ID, MotorType.kBrushless);
    private final SparkMax leftMotor = new SparkMax(Constants.Elevator.LEFT_MOTOR_ID, MotorType.kBrushless);

    private final DigitalInput topSwitch = new DigitalInput(Constants.Elevator.TOP_SENSOR_ID);
    private final DigitalInput bottomSwitch = new DigitalInput(Constants.Elevator.BOTTOM_SENSOR_ID);
    // Speed
    private static final double CorrectionInchesPerSecond = 5;
    // TODO Update this when we have a gear ratio
    // to make the motor rotate once (gear-ratio)
    private static final double MotorRotationsPerRevolution = 30;
    // the amount of times motor should spin to make it move one inch
    // TODO test and measure how many rotations for the elevator to lift one inch
    private static final double MotorRotationsPerInch = MotorRotationsPerRevolution * 5;
    // the amount of inches it should move to make the motor rotate once
    private static final double InchesPerMotorRotation = 1.0 / MotorRotationsPerInch;
    private static final double AllowedErrorInches = 2.0;

    private final SparkClosedLoopController pidControllerLeft = leftMotor.getClosedLoopController();
    private final RelativeEncoder encoder = leftMotor.getEncoder();

    private double currentHeight = encoder.getPosition() / MotorRotationsPerInch;

    private double targetInches = 0;

    public static Elevator get() {
        return instance;
    }

    @Override
    public void periodic() {
        // setHeight(targetInches); //This method sets the elevator without checking
        // sensors
        checkLimits(targetInches); // This method sets the elevator after checking sensors
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
                System.out.println("Elevator -> Down from Sensor");
            }
            // If you are trying to go up more: Stop Motor
            else if (targetInches > currentHeight) {
                leftMotor.stopMotor();
                System.out.println("THE ELEVATOR IS ON THE TOP SWITCH");
            }
        }
        // if the bottom switch is on
        else if (bottomSwitch.get()) {
            // if you want to go up: move normally
            if (targetInches >= currentHeight) {
                setHeight(target);
                System.out.println("Elevator -> Up from Sensor");
            }
            // if you want to go more down: stop motor
            else {
                rightMotor.stopMotor();
                System.out.println("THE ELEVATOR IS ON THE BOTTOM SWITCH");
            }
        }

        else {
            setHeight(target);
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

    public Command updateCommand(ElevatorState elevatorState) {
        return Commands.run(() -> {
            setTargetInches(elevatorState.getHeigtInches());
        }, this);
    }
}
