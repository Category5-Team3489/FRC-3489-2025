package frc.robot;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VortexTesting extends SubsystemBase {

    private static final VortexTesting instance = new VortexTesting();

    private final SparkFlex motor = new SparkFlex(1, MotorType.kBrushless);

    public static VortexTesting get() {
        return instance;
    }

    public Command runMotor(double speed) {
        return Commands.runOnce(() -> {
            motor.set(speed);
        });
    }

}
