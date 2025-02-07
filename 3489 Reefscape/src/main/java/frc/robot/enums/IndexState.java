package frc.robot.enums;

public enum IndexState {
    Outtake(-1, -1),
    Intake(1, 1),
    Stop(0, 0);

    private final double rightSpeedPercent;
    private final double leftSpeedPercent;

    private IndexState(double rightSpeedPercent, double leftSpeedPercent) {
        this.rightSpeedPercent = rightSpeedPercent;
        this.leftSpeedPercent = leftSpeedPercent;
    }

    public double getLeftSpeedPercent() {
        return leftSpeedPercent;
    }

    public double getRightSpeedPercent() {
        return rightSpeedPercent;
    }

}