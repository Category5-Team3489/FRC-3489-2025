package frc.robot.enums;

public enum OuttakeState {
    Outtake(-0.75),
    Intake(0.75),
    Stop(0);

    private final double speedPercent;

    private OuttakeState(double speedPercent) {
        this.speedPercent = speedPercent;
    }

    public double getSpeedPercent() {
        return speedPercent;
    }

}