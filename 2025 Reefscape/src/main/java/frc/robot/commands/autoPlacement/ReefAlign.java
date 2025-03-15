package frc.robot.commands.autoPlacement;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.enums.AlignState;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorLimelight;

public class ReefAlign extends Command {
    // TODO fix the constants
    private static double MaxStrafeMetersPerSecond = 0.5;
    private static double MaxDistanceMetersPerSecond = 1;
    private static double StrafeToleranceDegrees = 0.6;
    private static double DistanceToleranceDegrees = 0.25;
    private static Rotation2d TargetAngle = Rotation2d.fromDegrees(180);
    private static double SpeedLimiter = 0.5;
    private static double MaxOmegaDegreesPerSecond = 45;
    private static double TargetXSetpointDegrees = -5.72;
    private static double TargetYSetpointDegrees = -5.48;
    public static double FeedforwardMetersPerSecond = 0.02;

    private boolean hasHitStrafeSetpoint = false;

    private ElevatorLimelight limelight;
    private CommandSwerveDrivetrain drivetrain;
    // private ElevatorLimelight limelight;
    // private CommandSwerveDrivetrain drivetrain;
    private PIDController strafeController = new PIDController(0.15, 0, 0);
    private PIDController distanceController = new PIDController(0.25, 0, 0);

    private double xMetersPerSecond = 0;
    private double yMetersPerSecond = 0;

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
    // test robotrentric
    private final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    Command CommandSwerveDrive;

    private double drivetrainAngleRate = 0;
    private double drivetrainVelocityX = 0;
    private double drivetrainVelocityY = 0;

    public ReefAlign(CommandSwerveDrivetrain drivetrain, ElevatorLimelight limelight) {
        this.drivetrain = drivetrain;
        this.limelight = limelight;
        CommandSwerveDrive = drivetrain.applyRequest(() -> drive
                .withVelocityX(0)// this is avtually Y drives forward and back
                .withVelocityY(getDrivetrainVelocityY()) // this is actually X drives in
                .withRotationalRate(0));

        addRequirements(drivetrain, limelight);
    }

    @Override
    public void initialize() {
        // limelight.setDesiredPipeline(LimelightPipeline.MidRetroreflectiv
        System.out.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! TEST REEF ALIGN INITIALIZE");

        strafeController.setTolerance(StrafeToleranceDegrees);
        distanceController.setTolerance(DistanceToleranceDegrees);
    }

    private void printthings() {
        System.out.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! TEST REEF ALIGN INITIALIZE");

    }

    @Override
    public void execute() {

        // System.out.println("---------------------------------------------- TEST REEF
        // ALIGN");
        double targetX = limelight.getTargetX();
        double targetY = limelight.getTargetY();

        if (!Double.isNaN(targetX)) {
            yMetersPerSecond = -strafeController.calculate(targetX, TargetXSetpointDegrees);
            yMetersPerSecond = MathUtil.clamp(yMetersPerSecond, -MaxStrafeMetersPerSecond, MaxStrafeMetersPerSecond);
        } else {
            yMetersPerSecond = 0;
        }

        if (strafeController.atSetpoint() && !hasHitStrafeSetpoint) {
            hasHitStrafeSetpoint = true;
        }

        // if (hasHitStrafeSetpoint) {
        // xMetersPerSecond = distanceRateLimiter.calculate(WallSpeedMetersPerSecond);
        // }

        // drivetrain.driveFieldRelative(xMetersPerSecond, yMetersPerSecond,
        // SpeedLimiter, TargetAngle,
        // MaxOmegaDegreesPerSecond);
        if (targetX == AlignState.LeftCenter.getX() && targetY == AlignState.LeftCenter.getY()) {
            drivetrainVelocityX = 0;
            drivetrainVelocityY = 0;
            System.out.println("------don't move" + targetX);

        } else if (targetX > AlignState.Left.getX()) {
            drivetrainVelocityX = 0.2 * MaxSpeed;
            System.out.println("--------to the right---------" + targetX);

        } else {
            drivetrainVelocityX = 0.2 * MaxSpeed;
            System.out.println("--------to the left------" + targetX);

        }
        // todo fix the distance
        if (targetY > AlignState.Left.getY()) {
            drivetrainVelocityY = 0.2 * MaxSpeed;
            System.out.println("--------------------------------------------" + targetX);

        } else {
            drivetrainVelocityY = 0.2 * MaxSpeed;
            System.out.println("--------------------------------------------" + targetX);
        }
        CommandSwerveDrive.schedule();
    }

    private double getDrivetrainAngleRate() {
        return drivetrainAngleRate;
    }

    private double getDrivetrainVelocityX() {
        return drivetrainVelocityX;
    }

    private double getDrivetrainVelocityY() {
        return drivetrainVelocityY;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("---------------------------------------------- TEST REEF ALIGN END");

        drivetrain.applyRequest(() -> brake);
    }
}
