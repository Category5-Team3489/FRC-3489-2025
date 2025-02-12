package frc.robot.enums;

public enum IntakeRollerState {
    Intake(1),
    Outtake(-1),
    Stop(0);

    private final double speedPercent;

    private IntakeRollerState(double speedPercent) {
        this.speedPercent = speedPercent;
    }

    public double getSpeedPercent() {
        return speedPercent;
    }

}