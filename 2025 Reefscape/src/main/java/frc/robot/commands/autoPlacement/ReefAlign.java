// package frc.robot.commands.autoPlacement;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.CommandSwerveDrivetrain;
// import frc.robot.subsystems.ElevatorLimelight;

// public class ReefAlign extends Command {
// // TODO fix the constants
// private static double MaxStrafeMetersPerSecond = 0.5;
// private static double MaxDistanceMetersPerSecond = 1;
// private static double StrafeToleranceDegrees = 0.6;
// private static double DistanceToleranceDegrees = 0.25;
// private static Rotation2d TargetAngle = Rotation2d.fromDegrees(180);
// private static double SpeedLimiter = 0.5;
// private static double MaxOmegaDegreesPerSecond = 45;
// private static double TargetXSetpointDegrees = -5.72;
// private static double TargetYSetpointDegrees = -5.48;
// public static double FeedforwardMetersPerSecond = 0.02;

// private ElevatorLimelight limelight = ElevatorLimelight.get();
// private CommandSwerveDrivetrain drivetrain = CommandSwerveDrivetrain.get();
// private PIDController strafeController = new PIDController(0.15, 0, 0);
// private PIDController distanceController = new PIDController(0.25, 0, 0);

// private double xMetersPerSecond = 0;
// private double yMetersPerSecond = 0;

// public ReefAlign(ElevatorLimelight limelight, CommandSwerveDrivetrain
// drivetrain) {
// this.limelight = limelight;
// this.drivetrain = drivetrain;

// addRequirements(limelight, drivetrain);
// }

// @Override
// public void initialize() {
// limelight.setDesiredPipeline(LimelightPipeline.MidRetroreflective);

// strafeController.setTolerance(StrafeToleranceDegrees);
// distanceController.setTolerance(DistanceToleranceDegrees);
// }

// @Override
// public void execute() {
// if (!limelight.isActivePipeline(LimelightPipeline.MidRetroreflective) ||
// !drivetrain.isAroundTargetHeading()) {
// drivetrain.driveFieldRelative(0.0, 0.0, SpeedLimiter, TargetAngle,
// MaxOmegaDegreesPerSecond);
// return;
// }

// double targetX = limelight.getTargetX();
// if (!Double.isNaN(targetX)) {
// yMetersPerSecond = -strafeController.calculate(targetX,
// TargetXSetpointDegrees);
// yMetersPerSecond += Cat5.getSign(yMetersPerSecond) *
// FeedforwardMetersPerSecond;
// yMetersPerSecond = MathUtil.clamp(yMetersPerSecond,
// -MaxStrafeMetersPerSecond, MaxStrafeMetersPerSecond);
// // if (strafeController.atSetpoint()) {
// // yMetersPerSecond *= 0.3;
// // }
// } else {
// yMetersPerSecond = 0;
// }

// double targetY = limelight.getTargetY();
// if (!Double.isNaN(targetY)) {
// xMetersPerSecond = distanceController.calculate(targetY,
// TargetYSetpointDegrees);
// xMetersPerSecond += Cat5.getSign(xMetersPerSecond) *
// FeedforwardMetersPerSecond;
// xMetersPerSecond = MathUtil.clamp(xMetersPerSecond,
// -MaxDistanceMetersPerSecond,
// MaxDistanceMetersPerSecond);
// // if (distanceController.atSetpoint()) {
// // xMetersPerSecond *= 0.3;
// // }
// } else {
// xMetersPerSecond = 0;
// }

// drivetrain.driveFieldRelative(xMetersPerSecond, yMetersPerSecond,
// SpeedLimiter, TargetAngle,
// MaxOmegaDegreesPerSecond);
// }

// @Override
// public boolean isFinished() {
// return strafeController.atSetpoint() && distanceController.atSetpoint();
// }

// @Override
// public void end(boolean interrupted) {
// drivetrain.brakeTranslation();

// Cat5.print(getName() + " end");

// limelight.printTargetData();
// }
// }
