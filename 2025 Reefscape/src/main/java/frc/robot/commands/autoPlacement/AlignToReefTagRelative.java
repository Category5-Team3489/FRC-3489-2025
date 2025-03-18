// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoPlacement;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AlignToReefTagRelative extends Command {
    private PIDController xController, yController, rotController;
    private boolean isRightScore;
    private Timer dontSeeTagTimer, stopTimer;
    private CommandSwerveDrivetrain drivetrain;
    private double tagID = -1;

    private Command driveCommand;

    final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric();

    public AlignToReefTagRelative(boolean isRightScore, CommandSwerveDrivetrain drivebase) {
        xController = new PIDController(Constants.AutoAlign.X_REEF_ALIGNMENT_P, 0.0, 0); // Vertical movement
        yController = new PIDController(Constants.AutoAlign.Y_REEF_ALIGNMENT_P, 0.0, 0); // Horitontal movement
        rotController = new PIDController(Constants.AutoAlign.ROT_REEF_ALIGNMENT_P, 0, 0); // Rotation
        this.isRightScore = isRightScore;
        this.drivetrain = drivebase;
        addRequirements(drivebase);

    }

    @Override
    public void initialize() {
        this.stopTimer = new Timer();
        this.stopTimer.start();
        this.dontSeeTagTimer = new Timer();
        this.dontSeeTagTimer.start();

        rotController.setSetpoint(Constants.AutoAlign.ROT_SETPOINT_REEF_ALIGNMENT);
        rotController.setTolerance(Constants.AutoAlign.ROT_TOLERANCE_REEF_ALIGNMENT);

        xController.setSetpoint(Constants.AutoAlign.X_SETPOINT_REEF_ALIGNMENT);
        xController.setTolerance(Constants.AutoAlign.X_TOLERANCE_REEF_ALIGNMENT);

        yController
                .setSetpoint(isRightScore ? Constants.AutoAlign.Y_SETPOINT_REEF_ALIGNMENT
                        : -Constants.AutoAlign.Y_SETPOINT_REEF_ALIGNMENT);
        yController.setTolerance(Constants.AutoAlign.Y_TOLERANCE_REEF_ALIGNMENT);

        tagID = LimelightHelpers.getFiducialID("limelight-one");
    }

    @Override
    public void execute() {

        Boolean hastarget = LimelightHelpers.getTV("limelight-one");
        if (hastarget && LimelightHelpers.getFiducialID("limelight-one") == tagID) {
            this.dontSeeTagTimer.reset();

            double[] postions = LimelightHelpers.getBotPose_TargetSpace("limelight-one");
            SmartDashboard.putNumber("x", postions[2]);

            double xSpeed = xController.calculate(postions[2]);
            SmartDashboard.putNumber("xspee", xSpeed);
            double ySpeed = -yController.calculate(postions[0]);
            double rotValue = -rotController.calculate(postions[4]);

            driveCommand = drivetrain.applyRequest(() -> drive
                    .withVelocityX(xSpeed)
                    .withVelocityY(ySpeed)
                    .withRotationalRate(rotValue));

            driveCommand.schedule();

            if (!rotController.atSetpoint() ||
                    !yController.atSetpoint() ||
                    !xController.atSetpoint()) {
                stopTimer.reset();
            }
        } else {
            driveCommand.cancel();
        }

        SmartDashboard.putNumber("poseValidTimer", stopTimer.get());
        SmartDashboard.putBoolean("hasTarget", hastarget);

        Shuffleboard.getTab("Testing")
                .addBoolean("tv", () -> hastarget);

        Shuffleboard.getTab("Testing")
                .addDouble("poseValidTimer", () -> stopTimer.get());
    }

    @Override
    public void end(boolean interrupted) {
        driveCommand.cancel();
    }

    @Override
    public boolean isFinished() {
        // Requires the robot to stay in the correct position for 0.3 seconds, as long
        // as it gets a tag in the camera
        return this.dontSeeTagTimer.hasElapsed(Constants.AutoAlign.DONT_SEE_TAG_WAIT_TIME) ||
                stopTimer.hasElapsed(Constants.AutoAlign.POSE_VALIDATION_TIME);
    }
}