package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.enums.IntakeRollerState;

public class IntakeRoller extends SubsystemBase {
    private static final IntakeRoller instance = new IntakeRoller();

    // Devices (Vortex & CANrange sensor)
    private final SparkFlex motor = new SparkFlex(Constants.IntakeRoller.MOTOR_ID, MotorType.kBrushless);
    private final CANrange CANrange = new CANrange(Constants.IntakeRoller.SENSOR_ID,
            Constants.IntakeRoller.CAN_RANGE_CAN_BUS);

    CANrangeConfiguration configs = new CANrangeConfiguration(); // Configure the CANrange for basic use

    private double speed = 0; // Target Speed of Intake
    public boolean checkSensor = true; // Should the motor stop if the Sensor is triggered?

    public static IntakeRoller get() {
        return instance;
    }

    public IntakeRoller() {
        CANrange.getConfigurator().apply(configs); // Write these configs to the CANrange
    }

    public void testing() {
        motor.set(IntakeRollerState.IntakeTransfer.getSpeedPercent());
    }

    public double returnRange() {
        // Get Distance
        var distance = CANrange.getDistance();
        // refresh the value
        var currentDistance = distance.refresh();
        // converts the current distance to a double so that it can be returned
        double distanceDouble = currentDistance.getValueAsDouble();

        // Refresh and print these values
        // System.out.println("+++++++Double Distance is " +
        // currentDistance.toString());

        return distanceDouble; // TODO test that this returns the correct value with robot!
    }

    @Override
    public void periodic() {
        setIntake();
        checkSensor();

        // TODO TEST THIS
        // System.out.println("----------------------------------------------------SENSOR:"
        // + CANrange.getIsDetected());

        // System.out.println("________________________________" + returnRange());

    }

    // Set the speed of the motor to the global intake variable
    private void setIntake() {
        motor.set(speed);
    }

    // TODO Add shuffleboard logic to stop getting sensor values if sensor damaged
    // If current command requires sensor: stop motor when value is in range
    private void checkSensor() {
        if (checkSensor) {
            double sensorValue = returnRange();
            System.out.println("________________________________" + CANrange.getDistance());
            if (sensorValue <= Constants.IntakeRoller.SENSOR_RANGE) { // TODO Test/Update distance constant
                speed = IntakeRollerState.Stop.getSpeedPercent();
                System.out.println("STOP____________________________________________");
                // TODO DELETE!!
                // checkSensor = false;

            }
        }
    }

    // Update the global intake speed variable based on the input enum
    public Command updateSpeed(IntakeRollerState state, boolean checkSensor) {
        // this.checkSensor = checkSensor;
        // System.out.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!checkSensor:
        // " + checkSensor);
        return Commands.runOnce(() -> speed = state.getSpeedPercent());
    }

    public Command setSpeedCommand() {
        return Commands.runOnce(() -> motor.set(IntakeRollerState.IntakeTransfer.getSpeedPercent()), this);
    }

}
