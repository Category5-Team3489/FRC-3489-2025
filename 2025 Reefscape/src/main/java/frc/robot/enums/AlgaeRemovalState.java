package frc.robot.enums;

public enum AlgaeRemovalState {
    Outtake(-0.25),
    Intake(0.25),
    Stop(0);

    private final double speedPercent;

    private AlgaeRemovalState(double speedPercent) {
        this.speedPercent = speedPercent;
    }

    public double getSpeedPercent() {
        return speedPercent;
    }

}