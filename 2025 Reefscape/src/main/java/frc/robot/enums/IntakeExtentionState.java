package frc.robot.enums;

public enum IntakeExtentionState {
    // TODO Update these values
    HomePosition(0.28),
    IntakePosition(9);

    // TODO Update this name
    private final double value;

    private IntakeExtentionState(double value) {
        this.value = value;
    }

    public double getValue() {
        return value;
    }

}