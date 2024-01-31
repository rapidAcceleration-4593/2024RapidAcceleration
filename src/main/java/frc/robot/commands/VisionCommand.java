package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class VisionCommand extends Command {
    
    private final SwerveSubsystem swerve;
    private final PIDController distanceController;
    private final PIDController rotationController;

    public VisionCommand(SwerveSubsystem swerve) {
        // Initialize swerve for all movement
        this.swerve = swerve;

        // Initialize PIDController for distance control
        double kPTranslation = 0.75;
        double kITranslation = 0.0;
        double kDTranslation = 0.0;
        distanceController = new PIDController(kPTranslation, kITranslation, kDTranslation);
        distanceController.setSetpoint(36);

        // Initialize PIDController for rotation control
        double kPRotation = 1.0;
        double kIRotation = 0.0;
        double kDRotation = 0.0;
        rotationController = new PIDController(kPRotation, kIRotation, kDRotation);
        rotationController.setSetpoint(0.0);

        // Set deadband for distance & rotation PID Controllers
        distanceController.setTolerance(0.5);
        rotationController.setTolerance(0.5);
    }

    @Override
    public void execute() {
        // Values switched because camera is rotated 90 degrees
        double horizontalOffset = LimelightHelpers.getTY("");
        double verticalOffset = LimelightHelpers.getTX("");

        double cameraMountAngleDegrees = 0.0; // Degrees back Limelight is rotated from perfectly vertical
        double cameraLensHeightInches = 10.0; // Distance from center of Limelight lens to the floor
        double targetHeightInches = 60.0; // Distance from the target to the floor

        double angleToTargetDegrees = cameraMountAngleDegrees + verticalOffset;
        double angleToTargetRadians = angleToTargetDegrees * (Math.PI / 180.0);

        double distanceFromLimelightToTargetInches = (targetHeightInches - cameraLensHeightInches) / Math.tan(angleToTargetRadians); // Calculate Distance from Limelight to Target

        // PID Control Adjustments for distance & rotation
        double distanceAdjust = distanceController.calculate(distanceFromLimelightToTargetInches);
        double rotationAdjust = distanceController.calculate(horizontalOffset);

        // Swerve drive with both distance & rotation PID adjustments
        swerve.drive(new Translation2d(-distanceAdjust, 0.0), rotationAdjust, false);
    }
}
