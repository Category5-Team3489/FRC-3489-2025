package frc.robot.enums;

public enum ElevatorState {
    // TODO update these numbers
    Down(0.3), // 0.001
    L1(3), // 0.8
    L2(5.2), // 1.15
    L3(9.2), // 2.7
    Up(9.7); // 3.14

    private final double heightInches;

    private ElevatorState(double heightInches) {
        this.heightInches = heightInches;
    }

    public double getHeigtInches() {
        return heightInches;
    }

}