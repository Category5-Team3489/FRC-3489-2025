package frc.robot.enums;

public enum ElevatorState {
    // TODO update these numbers
    Down(0),
    L1(3),
    L2(5.2), // TESTED
    L3(8.5), // 8.7
    Up(8.8);

    private final double heightInches;

    private ElevatorState(double heightInches) {
        this.heightInches = heightInches;
    }

    public double getHeigtInches() {
        return heightInches;
    }

}