package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class VisionCommand extends Command {
    
    private final SwerveSubsystem swerve;
    private final double kPTranslation = 0.75;
    private final double kPRotation = 0.09;

    public VisionCommand(SwerveSubsystem swerve) {
        this.swerve = swerve;
    }

    @Override
    public void execute() {
        // Values switched because camera is rotated 90 degrees
        double horizontalOffset = LimelightHelpers.getTY("");
        double verticalOffset = LimelightHelpers.getTX("");

        double cameraMountAngleDegrees = 25.0; // Degrees back Limelight is rotated from perfectly vertical
        double cameraLensHeightInches = 20.0; // Distance from center of Limelight lens to the floor
        double targetHeightInches = 60.0; // Distance from the target to the floor

        double angleToTargetDegrees = cameraMountAngleDegrees + verticalOffset;
        double angleToTargetRadians = angleToTargetDegrees * (Math.PI / 180.0);

        // Calculate Distance from Limelight to Target
        double distanceFromLimelightToTargetInches = (targetHeightInches - cameraLensHeightInches) / Math.tan(angleToTargetRadians);

        // Might not work, but rotate to AprilTag
        // double rotateHeadOn = -horizontalOffset * kPRotation;
        // swerve.drive(new Translation2d(0.0, 0.0), rotateHeadOn, false);

        // Align the robot horizontally with AprilTag
        if (horizontalOffset < -2 || verticalOffset > 2) {
            swerve.drive(new Translation2d(0.0, -horizontalOffset * kPRotation), -horizontalOffset * kPRotation, false);
        } else {
            // Maintain 3-4 feet distance away from the AprilTag
            if (distanceFromLimelightToTargetInches < 36 && distanceFromLimelightToTargetInches != 0) {
                // Robot is too far away, drive forward
                swerve.drive(new Translation2d(-1 * kPTranslation, 0.0), 0.0, false);

                // double currentDistance = distanceFromLimelightToTargetInches;
                // double desiredDistance = 36;

                // double distanceError = desiredDistance - currentDistance;
                // double drivingAdjust = kPTranslation * distanceError;
                // Example: leftCommand += distanceAdjust;
            } else if (distanceFromLimelightToTargetInches > 48) {
                // Robot is too close, drive backward
                swerve.drive(new Translation2d(1 * kPTranslation, 0.0), 0.0, false);
            }
        }
    }
}
