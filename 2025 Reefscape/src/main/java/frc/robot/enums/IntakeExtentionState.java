package frc.robot.enums;

public enum IntakeExtentionState {
    // TODO Update these values
    HomePosition(0.4),
    MatchHome(3.59),
    IntakePosition(18);// 9

    // TODO Update this name
    private final double value;

    private IntakeExtentionState(double value) {
        this.value = value;
    }

    public double getValue() {
        return value;
    }

}