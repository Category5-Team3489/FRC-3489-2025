// package frc.robot.commands.autoPlacement;

// import java.util.function.DoubleSupplier;

// import com.ctre.phoenix6.swerve.SwerveRequest;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.controller.BangBangController;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.robot.Constants;
// import frc.robot.generated.TunerConstants;
// import frc.robot.subsystems.CommandSwerveDrivetrain;
// import frc.robot.subsystems.ElevatorLimelight;

// public class AutoPlace extends Command {
// private final ElevatorLimelight aprilLimelight = ElevatorLimelight.get();
// // private final CommandSwerveDrivetrain drivetrain =
// CommandSwerveDrivetrain.get();

// final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric();

// Command driveCommandForward = drivetrain.applyRequest(() -> drive
// .withVelocityX(getDrivetrainVelocityX())
// .withVelocityY(getDrivetrainVelocityY())
// .withRotationalRate(getDrivetrainAngleRate()));

// private Timer indexTimer = new Timer();
// private Timer shooterTimer = new Timer();

// private double drivetrainAngleRate = 0;
// private double drivetrainVelocityX = 0;
// private double drivetrainVelocityY = 0;

// // TODO Calculate this (MaxRadiansPerSecond)
// private double rotationSpeed = 0.05 *
// Constants.Drivetrain.MaxRadiansPerSecond;

// private final double targetXRange = 5;
// private final double maxYMeterRange = 4;
// private final double minYMeterRange = 0.2;

// public AutoPlace() {

// }

// @Override
// public void initialize() {
// double targetY = aprilLimelight.getTargetY();

// indexTimer.stop();
// shooterTimer.stop();

// indexTimer.reset();
// shooterTimer.reset();

// shooterTimer.start();

// driveCommandForward.schedule();
// }

// @Override
// public void execute() {

// if (driveCommandForward.isScheduled()) {
// System.out.println("Scheduled");
// } else {
// System.out.println("Not Scheduled");
// }

// double targetX = aprilLimelight.getTargetX();
// double targetY = aprilLimelight.getTargetY();
// double targetV = aprilLimelight.getTargetVisible();

// // Return if april tag is not visible
// if (targetV == 0) {
// drivetrainAngleRate = 0;
// drivetrainVelocityX = -0.5;
// drivetrainVelocityY = 0;
// System.out.println("Double.isNaN(targetX) || Double.isNaN(targetY)");
// return;
// }

// // Apriltag is visible
// if (Math.abs(targetX) < targetXRange) {
// driveCommandForward.cancel();

// } else if (targetX < 0) {
// drivetrainAngleRate = -rotationSpeed;
// driveCommandForward.schedule();
// } else if (targetX > 0) {
// drivetrainAngleRate = rotationSpeed;
// driveCommandForward.schedule();
// }

// }

// private double getDrivetrainAngleRate() {
// return drivetrainAngleRate;
// }

// private double getDrivetrainVelocityX() {
// return drivetrainVelocityX;
// }

// private double getDrivetrainVelocityY() {
// return drivetrainVelocityY;
// }

// @Override
// public void end(boolean interrupted) {
// driveCommandForward.cancel();
// System.out.println("Cancled");
// }

// @Override
// public boolean isFinished() {
// return indexTimer.hasElapsed(2);
// }
// }