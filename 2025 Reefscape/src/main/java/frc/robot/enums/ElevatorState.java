package frc.robot.enums;

public enum ElevatorState {
    // TODO update these numbers
    Down(0.3),
    L1(3),
    L2(5.2), // TESTED
    L3(9.2), // 8.7
    Up(9.7);

    private final double heightInches;

    private ElevatorState(double heightInches) {
        this.heightInches = heightInches;
    }

    public double getHeigtInches() {
        return heightInches;
    }

}