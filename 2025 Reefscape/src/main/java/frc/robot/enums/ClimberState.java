package frc.robot.enums;

public enum ClimberState {
    On(0.5),
    Off(0);

    private final double speed;

    private ClimberState(double speed) {
        this.speed = speed;
    }

    public double getSpeed() {
        return speed;
    }

}
