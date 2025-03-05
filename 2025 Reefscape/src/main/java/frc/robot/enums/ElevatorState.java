package frc.robot.enums;

public enum ElevatorState {
    // TODO update these numbers
    Down(0.001), // 0.3
    L1(0.8), // 3
    L2(1.15), // 5.2
    L3(2.7), // 9.2
    Up(3.14); // 9.7

    private final double heightInches;

    private ElevatorState(double heightInches) {
        this.heightInches = heightInches;
    }

    public double getHeigtInches() {
        return heightInches;
    }

}