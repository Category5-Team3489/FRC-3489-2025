package frc.robot.commands.autoPlacement;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorLimelight;

public class Align extends Command{

    private final ElevatorLimelight limelight = ElevatorLimelight.get();
    private CommandSwerveDrivetrain drivetrain;

    final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric();

    private Command driveCommandForward;

    private double drivetrainAngleRate = 0;
    private double drivetrainVelocityX = 0;
    private double drivetrainVelocityY = 0;

    private double rotationSpeed = 0.05 * Constants.Drivetrain.MaxRadiansPerSecond;
    private double translationSpeed = 0.5;

    private final double targetXRange = 5;
    private final double targetYRange = 5;

    private boolean xAlign = false;
    private boolean yAlign = false;


    private final double maxYMeterRange = 4;
    private final double minYMeterRange = 0.2;

    public Align(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;

        addRequirements(drivetrain);

    }

    @Override
    public void initialize() {
        driveCommandForward = drivetrain.applyRequest(() -> drive
            .withVelocityX(5)
            .withVelocityY(5)
            .withRotationalRate(5));

        //TODO Uncomment when velosity passed correctly: driveCommandForward.schedule();

    }

    @Override
    public void execute() {
        double targetX = limelight.getTargetX();
        double targetY = limelight.getTargetY();
        double targetV = limelight.getTargetVisible();


        //Return if Tag is not visible
        if (targetV == 0) {
            drivetrainAngleRate = 0;
            drivetrainVelocityX = -0.5;
            drivetrainVelocityY = 0;
            System.out.println("Tag is not Visible");
            return;
        }

        if (Math.abs(targetX) < targetXRange) {
            drivetrainVelocityX = 0;
            xAlign = true;
        } else if (targetX < 0) {
            drivetrainVelocityX = -translationSpeed;
            driveCommandForward.schedule();
        } else if (targetX > 0) {
            drivetrainAngleRate = translationSpeed;
            driveCommandForward.schedule();
        }

        if (Math.abs(targetY) < targetYRange) {
            drivetrainVelocityY = 0;
            yAlign = true;
        } else if (targetX < 0) {
            drivetrainAngleRate = -translationSpeed;
            driveCommandForward.schedule();
        } else if (targetX > 0) {
            drivetrainAngleRate = translationSpeed;
            driveCommandForward.schedule();
        }

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
    public boolean isFinished() {
        return xAlign && yAlign;
    }

    @Override
    public void end(boolean interrupted) {
        driveCommandForward.cancel();
        
       
    }

    
}
