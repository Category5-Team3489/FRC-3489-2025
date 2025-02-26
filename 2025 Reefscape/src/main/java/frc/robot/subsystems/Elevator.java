package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.enums.ElevatorState;

public class Elevator extends SubsystemBase {

    private static final Elevator instance = new Elevator();

    // TODO Add the left motor as a follower for right motor in Rev client
    // TODO Add the pid values for the right motor in the rev client
    private final SparkMax rightMotor;
    private final SparkMax leftMotor;

    private final RelativeEncoder encoder;

    private final SparkClosedLoopController pidControllerRight;

    // TODO Make sure we dont actually need to use the encoder in the code
    // private final AbsoluteEncoder encoder = rightMotor.getAbsoluteEncoder();
    // private final SparkAbsoluteEncoder encoder = rightMotor.getAbsoluteEncoder();

    private static final double CorrectionTicsPerSecond = 8192; // 4096

    // to make the motor rotate once (gear-ratio)
    private static final double gearRatio = 3;

    private final int sparkTicsPerRotation = 4096; // 2048 Cycles per Revolution (8192 Counts per Revolution)

    // TODO Measure (HeightChange from 1 rotation of final)
    // Height (in Tics) = Tics * Gear ratio * HeightChange from 1 rotation of final
    // gear

    // Encoder Ticks
    private double targetTics = 0;

    private double speed = 0;

    public static Elevator get() {
        return instance;
    }

    public Elevator() {
        rightMotor = new SparkMax(Constants.Elevator.RIGHT_MOTOR_ID, MotorType.kBrushless);
        leftMotor = new SparkMax(Constants.Elevator.LEFT_MOTOR_ID, MotorType.kBrushless);

        pidControllerRight = rightMotor.getClosedLoopController();

        encoder = rightMotor.getEncoder();

        Shuffleboard.getTab("Main")
                .addDouble("Right Encoder", () -> encoder.getPosition())
                .withSize(1, 1)
                .withPosition(7, 3);
    }

    @Override
    public void periodic() {
        // setElevator();
        // setHeight(); // This method sets the elevator without checking
        // System.out.println("*******************target position: " + targetTics);
        SmartDashboard.putNumber("Elevator Encoder", getEncoder());
        System.out.println("Endoder: " + encoder.getPosition());
        // setHeight();
        // System.out.println("Endoder: " +
        // leftMotor.getAbsoluteEncoder().getPosition());

        // System.out.println("*******************units of rotations?: " +
        // rightMotor.getAbsoluteEncoder().getPosition());
    }

    private double getEncoder() {
        return encoder.getPosition();
    }

    public void printEncoder() {
        System.out.println("Endoder: " + encoder.getPosition());
    }

    // Move the elevator to the correct height
    private void setHeight() {
        setTargetTics(targetTics);
        // double targetRotations = (targetTics * gearRatio) / sparkTicsPerRotation;
        // //Through Bore Math
        // double targetRotations = (targetTics / sparkTicsPerRotation); // Built in
        // encoder
        double targetRotations = 0; // 8.8

        pidControllerRight.setReference(targetRotations, ControlType.kPosition,
                ClosedLoopSlot.kSlot0);
        System.out.println("**************************************target rotation: " + targetRotations);
    }

    private void setTargetTics(double positionHeight) {
        targetTics = MathUtil.clamp(positionHeight,
                ElevatorState.Down.getHeigtInches(), ElevatorState.Up.getHeigtInches());
        // System.out.println("pos: " + positionHeight);
    }

    public Command adjustManualHeight(double adjustPercent) {
        return Commands.run(() -> {
            double deltaDegrees = adjustPercent * CorrectionTicsPerSecond *
                    Robot.kDefaultPeriod;
            setTargetTics(targetTics + deltaDegrees);
        }, this);
    }

    public Command updateCommand(ElevatorState elevatorState) {
        return Commands.run(() -> {
            setTargetTics(elevatorState.getHeigtInches());
        }, this);
    }

    // Manual Testing --------------------------------------------

    // Set the speed of the motor to the global outtake variable
    private void setElevator() {
        rightMotor.set(-speed);
        System.out.println("++++++++++++++++++++++=Speed: " + speed);

    }

    // Update the global outtake speed variable based on the input enum
    public Command updateSpeed() {
        return Commands.runOnce(() -> speed = -0.1);
    }

    // Other Manual-------------------------------------------------------
    public Command manualJoystick(double joystick) {
        return Commands.runOnce(() -> {
            rightMotor.set(joystick * 0.5);
            // System.out.println("RIGHT MOTOR SPEED: " + joystick * 0.5);
        });
    }
    // -------------------------------------------------------------------

}
