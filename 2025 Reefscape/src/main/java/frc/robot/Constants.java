// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.generated.TunerConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static class OperatorConstants {
        public static final int kDriverControllerPort = 0;
    }

    public static class Drivetrain {
        // TODO Calculate this
        // public static final int MaxRadiansPerSecond = 1;

        public static double MaxMetersPerSecond = 16.5 / 3.281;

        public static double DrivetrainHypotenuseMeters = Math.hypot(11.5 / 39.37,
                -11.5 / 39.37);

        public static double MaxRadiansPerSecond = MaxMetersPerSecond / -DrivetrainHypotenuseMeters;
    }

    public static class Elevator {
        public static final int RIGHT_MOTOR_ID = 13;
        public static final int LEFT_MOTOR_ID = 14;

        public static final int BOTTOM_SENSOR_ID = 1;

    }

    public static class Outtake {
        public static final int MOTOR_ID = 15;

        public static final int SENSOR_ID = 1;
    }

    // public static class IntakeRoller {
    // public static final int MOTOR_ID = 10;

    // public static final int SENSOR_ID = 0;

    // // TODO: Check - Not sure if this is correct
    // public static final String CAN_RANGE_CAN_BUS = "rio";

    // public static final double SENSOR_RANGE = 0.3;
    // }

    public static class IntakeActuator {
        public static final int MOTOR_ID = 9;
        public static final int SENSOR_ID = 10;
    }

    public static class Index {
        public static final int RIGHT_MOTOR_ID = 11;
        public static final int LEFT_MOTOR_ID = 12;
    }

    public static class AutoAlign {
        public static final double X_REEF_ALIGNMENT_P = 0.01;
        public static final double Y_REEF_ALIGNMENT_P = 0.01;
        public static final double ROT_REEF_ALIGNMENT_P = 0.01;

        public static final double ROT_SETPOINT_REEF_ALIGNMENT = 0;
        public static final double ROT_TOLERANCE_REEF_ALIGNMENT = 5;

        public static final double X_SETPOINT_REEF_ALIGNMENT = 40.33;
        public static final double X_TOLERANCE_REEF_ALIGNMENT = 5;

        public static final double Y_SETPOINT_REEF_ALIGNMENT = -3.9;
        public static final double Y_TOLERANCE_REEF_ALIGNMENT = 5;

        public static final double DONT_SEE_TAG_WAIT_TIME = 1;
        public static final double POSE_VALIDATION_TIME = 1;

    }

}
