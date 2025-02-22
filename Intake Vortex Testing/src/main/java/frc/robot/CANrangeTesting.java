package frc.robot;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CANrangeTesting extends SubsystemBase {

    private static final CANrangeTesting instance = new CANrangeTesting();

    private final CANrange CANrange = new CANrange(0, "rio");
    private final SparkFlex motor = new SparkFlex(1, MotorType.kBrushless);

    // CANrangeConfiguration configs = new CANrangeConfiguration(); // Configure the
    // CANrange for basic use

    private double speed = 0; // Target Speed of Intake
    private boolean checkSensor = false; // Should the motor stop if the Sensor is triggered?

    public static CANrangeTesting get() {
        return instance;
    }

    public CANrangeTesting() {
        // CANrange.getConfigurator().apply(configs); // Write these configs to the
        // CANrange
    }

    @Override
    public void periodic() {
        setIntake();
        checkSensor();
    }

    public double returnRange() {
        // Get Distance
        var distance = CANrange.getDistance();
        // refresh the value
        var currentDistance = distance.refresh();
        // converts the current distance to a double so that it can be returned
        double distanceDouble = currentDistance.getValueAsDouble();

        // Refresh and print these values
        // System.out.println("Distance is " + distance.refresh().toString());
        System.out.println("+++++++Double Distance is " + currentDistance.toString());

        return distanceDouble; // TODO test that this returns the correct value with robot!
    }

    // Set the speed of the motor to the global intake variable
    public void setIntake() {
        motor.set(speed);
    }

    // TODO Add shuffleboard logic to stop getting sensor values if sensor damaged
    // If current command requires sensor: stop motor when value is in range
    private void checkSensor() {
        if (checkSensor) {
            double sensorValue = returnRange();
            if (sensorValue <= 0.5) { // TODO Test/Update distance constant
                System.out.println("STOP____________________________________________");
                speed = 0;
            }
        }
    }

    // Update the global intake speed variable based on the input enum
    public Command updateSpeed(double speed, boolean checkSensor) {
        this.checkSensor = checkSensor;
        return Commands.runOnce(() -> this.speed = speed);
    }

}
