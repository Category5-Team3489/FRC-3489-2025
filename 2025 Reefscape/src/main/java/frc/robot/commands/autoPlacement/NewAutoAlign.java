package frc.robot.commands.autoPlacement;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorLimelight;

public class NewAutoAlign extends Command {
    private static double ProportionalGain = 1; // 0.18 //1 worked!!!
    private static double MaxStrafeMetersPerSecond = 0.8;
    private static double StrafeToleranceDegrees = 1;
    // private static Rotation2d TargetAngle = Rotation2d.fromDegrees(180);
    private static double SpeedLimiter = 0.5;
    // private static double MaxOmegaDegreesPerSecond = 180; // 90
    private static double TargetXSetpointDegrees = -17.83;
    private static double TargetYSetpointDegrees = 4.38;

    private static double WallSpeedMetersPerSecond = -0.5;
    private static double WallTimeoutSeconds = 2;

    private Timer wallTimer = new Timer();

    private final ElevatorLimelight limelight;
    private final CommandSwerveDrivetrain drivetrain;
    private PIDController strafeController = new PIDController(ProportionalGain, 0, 0);
    private SlewRateLimiter distanceRateLimiter = new SlewRateLimiter(5);

    private Command driveCommandForward;

    final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric();

    private double xMetersPerSecond = 0;
    private double yMetersPerSecond = 0;

    private boolean hasHitStrafeSetpoint = false;

    public NewAutoAlign(ElevatorLimelight limelight, CommandSwerveDrivetrain drivetrain) {
        this.limelight = limelight;
        this.drivetrain = drivetrain;

        addRequirements(limelight, drivetrain);
    }

    @Override
    public void initialize() {
        strafeController.setTolerance(StrafeToleranceDegrees);
    }

    @Override
    public void execute() {
        // if (!limelight.isActivePipeline(LimelightPipeline.HighRetroreflective)) {
        // drivetrain.driveFieldRelative(xMetersPerSecond, yMetersPerSecond,
        // SpeedLimiter, TargetAngle,
        // MaxOmegaDegreesPerSecond);
        // return;
        // }

        System.out.println("EXECUTE!!!!!");

        double currentX = limelight.getTargetX();
        double currentY = limelight.getTargetY();

        if (!Double.isNaN(currentY)) {
            yMetersPerSecond = -strafeController.calculate(currentY, TargetYSetpointDegrees);
            yMetersPerSecond = MathUtil.clamp(yMetersPerSecond, -MaxStrafeMetersPerSecond, MaxStrafeMetersPerSecond);
        } else {
            yMetersPerSecond = 0;
        }

        if (!Double.isNaN(currentX)) {
            xMetersPerSecond = -strafeController.calculate(currentX, TargetXSetpointDegrees);
            xMetersPerSecond = MathUtil.clamp(xMetersPerSecond, -MaxStrafeMetersPerSecond, MaxStrafeMetersPerSecond);
        } else {
            xMetersPerSecond = 0;
        }

        if (strafeController.atSetpoint() && !hasHitStrafeSetpoint) {
            hasHitStrafeSetpoint = true;

            wallTimer.restart();
        }

        // if (hasHitStrafeSetpoint) {
        // xMetersPerSecond = distanceRateLimiter.calculate(WallSpeedMetersPerSecond);
        // }

        // drivetrain.driveFieldRelative(xMetersPerSecond, yMetersPerSecond,
        // SpeedLimiter, TargetAngle,
        // MaxOmegaDegreesPerSecond);

        driveCommandForward = drivetrain.applyRequest(() -> {
            return drive
                    .withVelocityX(xMetersPerSecond * SpeedLimiter)
                    .withVelocityY(yMetersPerSecond * SpeedLimiter);
        });

        driveCommandForward.schedule();
    }

    @Override
    public boolean isFinished() {
        System.out.println("Timer" + wallTimer.get());
        return wallTimer.hasElapsed(WallTimeoutSeconds);
    }

    @Override
    public void end(boolean interrupted) {
        driveCommandForward = drivetrain.applyRequest(() -> {
            return drive
                    .withVelocityX(0)
                    .withVelocityY(0);
        });

        System.out.println("STOP");

        driveCommandForward.schedule();
    }
}
