package frc.robot.enums;

public enum OuttakeState {
    Outtake(-0.25),
    Intake(0.25),
    Stop(0);

    private final double speedPercent;

    private OuttakeState(double speedPercent) {
        this.speedPercent = speedPercent;
    }

    public double getSpeedPercent() {
        return speedPercent;
    }

}