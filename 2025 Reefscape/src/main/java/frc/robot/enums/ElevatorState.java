package frc.robot.enums;

public enum ElevatorState {
    Down(0.3), 
    L1(3), 
    L2(5.2), 
    L3(9.2), 
    Up(9.7); 

    private final double height;

    private ElevatorState(double height) {
        this.height = height;
    }

    public double getHeigt() {
        return height;
    }

}


