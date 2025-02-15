package frc.robot.enums;

public enum ElevatorState {
    // Test the numbers

    Down(0),
    L1(18),
    L2(.03),
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