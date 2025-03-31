package frc.robot.commands.autoPlacement;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.enums.AlignState;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorLimelight;

public class Align extends Command {

    private final ElevatorLimelight limelight = ElevatorLimelight.get();
    private CommandSwerveDrivetrain drivetrain;

    final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric();

    private Command driveCommandForward;

    private double drivetrainAngleRate = 0;
    private double drivetrainVelocityX;
    private double drivetrainVelocityY;

    private double rotationSpeed = 0.05 * Constants.Drivetrain.MaxRadiansPerSecond;
    private double translationSpeed = 0.2;

    private final double targetXRange = 5;
    private final double targetYRange = 5;

    private boolean xAlign = false;
    private boolean yAlign = false;

    private final double maxYMeterRange = 4;
    private final double minYMeterRange = 0.2;

    public boolean isPressed = true;

    public Align(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;

        addRequirements(drivetrain);

    }

    @Override
    public void initialize() {
        driveCommandForward = drivetrain.applyRequest(() -> {
            System.out.println("X: " + getDrivetrainVelocityX().getAsDouble());
            System.out.println("Y: " + getDrivetrainVelocityY().getAsDouble());
            System.out.println("Angle: " + getDrivetrainAngleRate().getAsDouble());

            return drive
                    .withVelocityX(getDrivetrainVelocityX().getAsDouble())
                    .withVelocityY(getDrivetrainVelocityY().getAsDouble())
                    .withRotationalRate(getDrivetrainAngleRate().getAsDouble());
        });

        // driveCommandForward.schedule();

        // TODO Uncomment when velosity passed correctly:
        // driveCommandForward.schedule();

    }

    @Override
    public void execute() {
        double currentX = limelight.getTargetX();
        double currentY = limelight.getTargetY();
        double targetV = limelight.getTargetVisible();
        double currentA = limelight.getTargetArea();

        // System.out.println("TEST");

        // Return if Tag is not visible
        if (targetV == 0) {
            // drivetrainAngleRate = 0;
            // drivetrainVelocityX = 0;
            // drivetrainVelocityY = 0;
            System.out.println("Tag is not Visible");
            return;
        }

        if (currentX > 4) { // 17
            drivetrainVelocityY = 0;
            xAlign = true;
            driveCommandForward.schedule();

            System.out.println("X Aligned, current x: " + currentX);
        } else if (currentX < 4) {
            drivetrainVelocityY = translationSpeed;
            System.out.println("X -> -speed ---- " + currentX + "Velosity: " + drivetrainVelocityY);
            driveCommandForward.schedule();

        } else if (currentX > 4) {
            drivetrainVelocityY = -translationSpeed;
            System.out.println("X -> +speed ---- " + currentX + "Velosity: " + drivetrainVelocityY);
            driveCommandForward.schedule();

        }

        // if (currentY > -1.5) {// -9
        // drivetrainVelocityX = 0;
        // xAlign = true;
        // driveCommandForward.schedule();

        // System.out.println("Y Aligned, current y: " + currentY);
        // } else if (currentY < -1.5) {
        // drivetrainVelocityX = translationSpeed;
        // System.out.println("Y -> -speed ---- " + currentY + "Velosity: " +
        // drivetrainVelocityX);
        // driveCommandForward.schedule();
        // } else if (currentY > -1.5) {
        // drivetrainVelocityX = -translationSpeed;
        // System.out.println("Y -> +speed ---- " + currentY + "Velosity: " +
        // drivetrainVelocityX);
        // driveCommandForward.schedule();
        // }

        if (currentA > 16) {// at 14 it could see it
            drivetrainVelocityX = 0;
            xAlign = true;
            driveCommandForward.schedule();

            System.out.println("Y Aligned, current y: " + currentY);
        } else if (currentY < 16) {
            drivetrainVelocityX = translationSpeed;
            System.out.println("Y -> -speed ---- " + currentY + "Velosity: " + drivetrainVelocityX);
            driveCommandForward.schedule();
        } else if (currentY > 16) {
            drivetrainVelocityX = -translationSpeed;
            System.out.println("Y -> +speed ---- " + currentY + "Velosity: " + drivetrainVelocityX);
            driveCommandForward.schedule();
        }
    }

    private DoubleSupplier getDrivetrainAngleRate() {
        System.out.println("drive angle rate: " + drivetrainAngleRate);
        return () -> drivetrainAngleRate;
    }

    private DoubleSupplier getDrivetrainVelocityX() {
        System.out.println("drive x rate: " + drivetrainVelocityX);

        return () -> drivetrainVelocityX;
    }

    private DoubleSupplier getDrivetrainVelocityY() {
        System.out.println("drive y rate: " + drivetrainVelocityY);

        return () -> drivetrainVelocityY;
    }

    // @Override
    // public boolean isFinished() {

    // // return xAlign && yAlign;
    // return xAlign;
    // }

    @Override
    public void end(boolean interrupted) {

        if (isPressed) {
            return;
        } else {
            driveCommandForward.cancel();
        }

        driveCommandForward.cancel();

    }

}
