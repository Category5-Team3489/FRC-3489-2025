package frc.robot.commands.autoPlacement;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorLimelight;

public class NewAutoAlign extends Command {
    private static double ProportionalGain = 0.1; // 0.18
    private static double MaxStrafeMetersPerSecond = 0.8;
    private static double MaxAngleMetersPerSecond = 0.5;

    private static double StrafeToleranceDegrees = 1.5;
    private static double DistanceToleranceDegrees = 1.5;

    // private static Rotation2d TargetAngle = Rotation2d.fromDegrees(180);
    private static double SpeedLimiter = 0.5;
    // private static double MaxOmegaDegreesPerSecond = 180; // 90
    private static double TargetXSetpointDegrees = -17.83;
    private static double TargetYSetpointDegrees = 4.38;
    private static double TargetAnglSetpointDegrees = 88; // ?
    private static Rotation2d TargetAngle = Rotation2d.fromDegrees(180);

    private static double WallSpeedMetersPerSecond = -0.5;
    private static double WallTimeoutSeconds = 2;

    private Timer wallTimer = new Timer();

    private final ElevatorLimelight limelight;
    private final CommandSwerveDrivetrain drivetrain;
    private PIDController strafeController = new PIDController(ProportionalGain, 0, 0);
    private PIDController distanceController = new PIDController(ProportionalGain, 0, 0);
    private SlewRateLimiter distanceRateLimiter = new SlewRateLimiter(5);

    private Command driveCommandForward;

    final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric();

    private double xMetersPerSecond = 0;
    private double yMetersPerSecond = 0;
    private double angleMetersPerSecond = 0;

    private double isTagVisible;

    private boolean hasHitStrafeSetpoint = false;

    public NewAutoAlign(ElevatorLimelight limelight, CommandSwerveDrivetrain drivetrain) {
        this.limelight = limelight;
        this.drivetrain = drivetrain;

        addRequirements(limelight);
    }

    @Override
    public void initialize() {
        strafeController.setTolerance(StrafeToleranceDegrees);
        distanceController.setTolerance(DistanceToleranceDegrees);
    }

    @Override
    public void execute() {

        // System.out.println("TS!!!!!");

        double currentX = limelight.getTargetX();
        double currentY = limelight.getTargetY();
        double currentAngle = limelight.getTargetS();

        isTagVisible = limelight.getTargetVisible();

        // System.out.println("Limelight: " + currentAngle);

        if (!Double.isNaN(currentY)) {
            yMetersPerSecond = -strafeController.calculate(currentY, TargetYSetpointDegrees);
            yMetersPerSecond = MathUtil.clamp(yMetersPerSecond, -MaxStrafeMetersPerSecond, MaxStrafeMetersPerSecond);
        } else {
            yMetersPerSecond = 0;
        }

        if (!Double.isNaN(currentX)) {
            xMetersPerSecond = -distanceController.calculate(currentX,
                    TargetXSetpointDegrees);
            xMetersPerSecond = MathUtil.clamp(xMetersPerSecond,
                    -MaxStrafeMetersPerSecond, MaxStrafeMetersPerSecond);
        } else {
            xMetersPerSecond = 0;
        }

        if (!Double.isNaN(currentAngle)) {
            angleMetersPerSecond = -strafeController.calculate(currentAngle, TargetAnglSetpointDegrees);
            angleMetersPerSecond = MathUtil.clamp(angleMetersPerSecond, -MaxAngleMetersPerSecond,
                    MaxAngleMetersPerSecond);
        } else {
            angleMetersPerSecond = 0;
        }

        if (strafeController.atSetpoint() && !hasHitStrafeSetpoint) {
            hasHitStrafeSetpoint = true;

            wallTimer.restart();
        }

        driveCommandForward = drivetrain.applyRequest(() -> {
            return drive
                    .withVelocityX(xMetersPerSecond * SpeedLimiter)
                    .withVelocityY(yMetersPerSecond * SpeedLimiter)
                    .withRotationalRate(angleMetersPerSecond * SpeedLimiter);
        });

        // System.out.println("Velosity Y: " + yMetersPerSecond * SpeedLimiter);

        driveCommandForward.schedule();
    }

    @Override
    public boolean isFinished() {
        // System.out.println("Timer" + wallTimer.get());
        // return wallTimer.hasElapsed(WallTimeoutSeconds);

        // return strafeController.atSetpoint() && distanceController.atSetpoint();
        // System.out.println("!!!!!!!!!!!!!!IS FINISHED!!!!!!!!!!!!!!!!!!!!");
        return (strafeController.atSetpoint() && distanceController.atSetpoint()) || isTagVisible == 0;
    }

    // @Override
    // public void end(boolean interrupted) {
    // driveCommandForward = drivetrain.applyRequest(() -> {
    // return drive
    // .withVelocityX(0)
    // .withVelocityY(0);
    // });

    // System.out.println("------------------STOP");

    // driveCommandForward.withTimeout(10).schedule();
    // }

    @Override
    public void end(boolean interrupted) {
        driveCommandForward.cancel();

    }
}
