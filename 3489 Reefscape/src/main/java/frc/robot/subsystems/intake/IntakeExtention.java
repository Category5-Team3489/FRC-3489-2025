package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeExtention extends SubsystemBase{

    private final SparkMax motor = new SparkMax(Constants.IntakeActuator.MOTOR_ID, MotorType.kBrushless);

    //TODO update to correct sensor type
    private final DigitalInput sensor = new DigitalInput(40);

    private static final double CorrectionInchesPerSecond = 5;
    // TODO Update this when we have a gear ratio
    // to make the motor rotate once (gear-ratio)
    private static final double MotorRotationsPerRevolution = 10;
    // the amount of times motor should spin to make it move one inch
    private static final double MotorRotationsPerInch = MotorRotationsPerRevolution / 360.0;
    // the amount of inches it should move to make the motor rotate once
    private static final double InchesPerMotorRotation = 1.0 / MotorRotationsPerInch;
    private static final double AllowedErrorInches = 2.0;

    private final SparkClosedLoopController pidControllerLeft = motor.getClosedLoopController();
    private final RelativeEncoder encoder = motor.getEncoder();

    // TODO do the math correctly after getting the actual numbers
    private double currentHeight = encoder.getPosition() * 10;

    private double targetInches = 0;
}
