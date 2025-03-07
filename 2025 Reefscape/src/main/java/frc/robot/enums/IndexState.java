package frc.robot.enums;

public enum IndexState {
    Outtake(-0.25),
    Intake(0.25),
    Stop(0);

    private final double speedPercent;

    private IndexState(double speedPercent) {
        this.speedPercent = speedPercent;
    }

    public double getSpeedPercent() {
        return speedPercent;
    }

}