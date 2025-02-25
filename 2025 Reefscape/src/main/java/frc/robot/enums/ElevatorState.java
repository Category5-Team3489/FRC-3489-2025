package frc.robot.enums;

public enum ElevatorState {
    // TODO update these numbers
    Down(0),
    L1(2),
    L2(27),
    L3(48),
    Up(55);

    private final double heightInches;

    private ElevatorState(double heightInches) {
        this.heightInches = heightInches;
    }

    public double getHeigtInches() {
        return heightInches;
    }

}