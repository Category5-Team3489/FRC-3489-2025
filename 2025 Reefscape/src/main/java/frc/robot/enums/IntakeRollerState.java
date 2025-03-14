package frc.robot.enums;

public enum IntakeRollerState {
    IntakeCollect(-0.1), // -0.25
    IntakeTransfer(-0.5),
    Outtake(-0.5),
    Stop(0);

    private final double speedPercent;

    private IntakeRollerState(double speedPercent) {
        this.speedPercent = speedPercent;
    }

    public double getSpeedPercent() {
        return speedPercent;
    }

}