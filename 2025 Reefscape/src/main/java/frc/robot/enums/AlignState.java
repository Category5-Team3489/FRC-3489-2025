package frc.robot.enums;

public enum AlignState {
    Left(41.9, -3.32), // 0.001
    Right(0.32, -8.6), // 0.8
    LeftCenter(35.49, -7.80);

    private final double tx;
    private final double ty;

    private AlignState(double tx, double ty) {
        this.tx = tx;
        this.ty = ty;

    }

    public double getX() {
        return tx;
    }

    public double getY() {
        return ty;
    }

}
