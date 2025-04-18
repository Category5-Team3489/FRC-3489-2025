package frc.robot.commands.autoPlacement;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeLimelight;

public class RightAutoAlign extends Command {
    // PID Values
    private static double ProportionalGain = 0.1; // 0.18

    // Max Spped
    private static double MaxStrafeMetersPerSecond = 0.8;
    private static double MaxAngleMetersPerSecond = 0.5;

    // The range it can be within cancle
    private static double StrafeToleranceDegrees = 0.025;
    private static double DistanceToleranceDegrees = 0.025;

    // Limit the speed
    private static double SpeedLimiter = 0.5;

    // The limelight setpoint values
    private static double TargetXSetpointDegrees = -0.3; // -0.25//-0.175
    private static double TargetYSetpointDegrees = -1.10; // 0.67
    // private static double TargetAnglSetpointDegrees = 4.7; // ?

    private Timer wallTimer = new Timer();

    // Declare the subsystems
    private final IntakeLimelight limelight;
    private final CommandSwerveDrivetrain drivetrain;

    // Declare PID Controllers
    private PIDController strafeController = new PIDController(ProportionalGain, 0, 0);
    private PIDController distanceController = new PIDController(ProportionalGain, 0, 0);

    // Declare Drive Command
    private Command driveCommandForward;
    final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric();

    // Declare drive velosities
    private double xMetersPerSecond = 0;
    private double yMetersPerSecond = 0;
    private double angleMetersPerSecond = 0;

    // Declare Tag Visible
    private double isTagVisible;

    private boolean hasHitStrafeSetpoint = false;

    public RightAutoAlign(IntakeLimelight limelight, CommandSwerveDrivetrain drivetrain) {
        this.limelight = limelight;
        this.drivetrain = drivetrain;

        addRequirements(limelight);
    }

    @Override
    public void initialize() {
        // Set Tolerance for PID controllers
        strafeController.setTolerance(StrafeToleranceDegrees);
        distanceController.setTolerance(DistanceToleranceDegrees);
    }

    @Override
    public void execute() {
        // Get limelight values
        double currentX = limelight.getTargetX();
        double currentY = limelight.getTargetY();
        double currentAngle = limelight.getTargetS();
        isTagVisible = limelight.getTargetVisible();

        // System.out.println(currentAngle);

        // Horizontal Alignment
        if (!Double.isNaN(currentY)) { // If the tag is Visible
            yMetersPerSecond = strafeController.calculate(currentY, TargetYSetpointDegrees);
            yMetersPerSecond = MathUtil.clamp(yMetersPerSecond, -MaxStrafeMetersPerSecond, MaxStrafeMetersPerSecond);
        } else {
            yMetersPerSecond = 0;
        }

        // Verticle Alignment
        if (!Double.isNaN(currentX)) { // If the tag is Visible
            xMetersPerSecond = distanceController.calculate(currentX,
                    TargetXSetpointDegrees);
            xMetersPerSecond = MathUtil.clamp(xMetersPerSecond,
                    -MaxStrafeMetersPerSecond, MaxStrafeMetersPerSecond);
        } else {
            xMetersPerSecond = 0;
        }

        // Angular Alignment
        if (!Double.isNaN(currentAngle)) { // If the tag is Visible
            // angleMetersPerSecond = strafeController.calculate(currentAngle,
            // TargetAnglSetpointDegrees);
            angleMetersPerSecond = MathUtil.clamp(angleMetersPerSecond, -MaxAngleMetersPerSecond,
                    MaxAngleMetersPerSecond);
        } else {
            angleMetersPerSecond = 0;
        }

        // I dont think we need this
        if (strafeController.atSetpoint() && !hasHitStrafeSetpoint) {
            hasHitStrafeSetpoint = true;

            wallTimer.restart();
        }

        // Set up the drive command
        driveCommandForward = drivetrain.applyRequest(() -> {
            return drive
                    .withVelocityX(xMetersPerSecond * SpeedLimiter)
                    .withVelocityY(yMetersPerSecond * SpeedLimiter);
            // .withRotationalRate(angleMetersPerSecond * SpeedLimiter);
        });

        // Schedule the drive command
        driveCommandForward.schedule();
    }

    @Override
    public boolean isFinished() {
        // System.out.println("!!!!!!!!!!!!!!IS FINISHED!!!!!!!!!!!!!!!!!!!!");
        return (strafeController.atSetpoint() && distanceController.atSetpoint()) || isTagVisible == 0;
    }

    @Override
    public void end(boolean interrupted) {
        driveCommandForward.cancel();

    }
}
