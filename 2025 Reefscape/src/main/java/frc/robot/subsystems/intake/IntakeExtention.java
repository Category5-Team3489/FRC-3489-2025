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
import frc.robot.enums.IntakeExtentionState;

public class IntakeExtention extends SubsystemBase {

    private static final IntakeExtention instance = new IntakeExtention();

    // TODO Update rev client values
    private final SparkMax motor = new SparkMax(Constants.IntakeActuator.MOTOR_ID, MotorType.kBrushless);
    private final SparkClosedLoopController pidController = motor.getClosedLoopController();

    private final RelativeEncoder encoder = motor.getEncoder();

    // TODO update to correct sensor type
    private final DigitalInput sensor = new DigitalInput(Constants.IntakeActuator.SENSOR_ID);

    private static final double CorrectionDegreesPerSecond = 10; // the speed

    // Gear ratio (motor rotates 30 times for one revolution of the actuator)
    private static final double gearRatio = 30;
    private final int sparkTicsPerRotation = 4096;

    // Intake is at its target angle if the error is within plus or minus this value
    private static final double AllowedErrorTics = 2.0;

    private double targetTics = 0;

    public static IntakeExtention get() {
        return instance;
    }

    private double getEncoder() {
        return encoder.getPosition();
    }

    // Move the intake to the correct position
    private void setPosition() {
        pidController.setReference(targetTics, ControlType.kPosition, ClosedLoopSlot.kSlot0);
        System.out.println("INTAKE: Target Tics = " + targetTics);
    }

    private void setTarget(double targetPosition) {
        targetTics = MathUtil.clamp(targetPosition,
                IntakeExtentionState.HomePosition.getValue(), IntakeExtentionState.IntakePosition.getValue());
    }

    public Command adjustManualHeight(double adjustPercent) {
        return Commands.run(() -> {
            double deltaDegrees = adjustPercent * CorrectionDegreesPerSecond * Robot.kDefaultPeriod;
            setTarget(targetTics + deltaDegrees);
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
        setPosition();
    }

    public Command manualJoystick(double joystick) {
        return Commands.runOnce(() -> {
            motor.set(joystick * 0.3);
        });
    }

}
