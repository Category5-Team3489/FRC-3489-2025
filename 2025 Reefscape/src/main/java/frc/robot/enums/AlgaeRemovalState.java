package frc.robot.enums;

public enum AlgaeRemovalState {
    PushUp(0.25),
    PushDown(-0.25),
    Stop(0);

    private final double speedPercent;

    private AlgaeRemovalState(double speedPercent) {
        this.speedPercent = speedPercent;
    }

    public double getSpeedPercent() {
        return speedPercent;
    }

}