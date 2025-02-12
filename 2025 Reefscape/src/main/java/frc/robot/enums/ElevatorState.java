package frc.robot.enums;

public enum ElevatorState {
    Down(0),
    L1(18),
    L2(31.875),
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