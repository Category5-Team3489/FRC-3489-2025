package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorLimelight extends SubsystemBase {
    private static ElevatorLimelight instance = new ElevatorLimelight();

    public static ElevatorLimelight get() {
        return instance;
    }

    // Devices
    // private final NetworkTable limelight =
    // NetworkTableInstance.getDefault().getTable("limelight-shooter");
    private final NetworkTable limelight;

    // Used to determine how long
    private Timer timer = new Timer();

    // Variables
    private final NetworkTableEntry tagIdEntry;
    private final NetworkTableEntry targetXEntry;
    private final NetworkTableEntry targetYEntry;
    private final NetworkTableEntry targetAreaEntry;
    private final NetworkTableEntry targetVisibleEntry;
    private final NetworkTableEntry targetAngleEntry;

    private double lastTargetX;
    private double lastTargetY;
    private double lastTargetS;

    private ElevatorLimelight() {

        limelight = NetworkTableInstance.getDefault().getTable("limelight-one");

        tagIdEntry = limelight.getEntry("tid");
        targetXEntry = limelight.getEntry("tx");
        targetYEntry = limelight.getEntry("ty");
        targetAreaEntry = limelight.getEntry("ta");
        targetVisibleEntry = limelight.getEntry("tv");
        targetAngleEntry = limelight.getEntry("ts");

        Shuffleboard.getTab("Main")
                .addDouble("Tag", () -> getTagId())
                .withSize(1, 1)
                .withPosition(7, 2);

        Shuffleboard.getTab("Testing")
                .addDouble("Limelight X", () -> getTargetX())
                .withSize(1, 1)
                .withPosition(5, 2);

        Shuffleboard.getTab("Testing")
                .addDouble("Limelight Y", () -> getTargetY())
                .withSize(1, 1)
                .withPosition(2, 2);
    }

    private boolean isTagVisible() {
        return targetVisibleEntry.getInteger(0) == 1;
    }

    public long getTagId() {
        return tagIdEntry.getInteger(-1);
    }

    public double getTargetX() {
        if (isTagVisible()) {
            lastTargetX = targetXEntry.getDouble(Double.NaN);
        }
        return lastTargetX;
    }

    public double getTargetY() {
        if (isTagVisible()) {
            lastTargetY = targetYEntry.getDouble(Double.NaN);
        }
        return lastTargetY;
    }

    public double getTargetS() {
        if (isTagVisible()) {
            lastTargetS = targetAngleEntry.getDouble(Double.NaN);
        }
        return lastTargetS;
    }

    public double getTargetArea() {
        return targetAreaEntry.getDouble(Double.NaN);
    }

    public long getTargetVisible() {
        if (isTagVisible()) {
            timer.stop();
            timer.reset();
            return 1;
        }
        timer.start();
        if (timer.hasElapsed(0.5)) {
            return 0;
        }
        return 1;
    }
}