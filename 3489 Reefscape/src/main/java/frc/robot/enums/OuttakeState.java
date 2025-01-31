package frc.robot.enums;

public enum OuttakeState {
    Outtake(1),
    Intake(-1),
    Stop(0);

    private final double speedPercent;

    private OuttakeState(double speedPercent) {
        this.speedPercent = speedPercent;
    }

    public double getSpeedPercent() {
        return speedPercent;
    }

}