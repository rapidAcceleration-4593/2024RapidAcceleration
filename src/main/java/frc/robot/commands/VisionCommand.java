package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class VisionCommand extends Command {

    private final SwerveSubsystem swerve;
    private final PIDController rotationController;
    private final PIDController translationController;

    private static final int targetID = 1;

    public VisionCommand(SwerveSubsystem swerve) {
        // Initialize Swerve
        this.swerve = swerve;

        // Initalize Translation PID Controller
        double kPTranslation = 0.0;
        double kITranslation = 0.0;
        double KDTranslation = 0.0;
        translationController = new PIDController(kPTranslation, kITranslation, KDTranslation);
        translationController.setSetpoint(0.0);
        translationController.setTolerance(0.5);

        // Initialize Rotation PID Controller
        double kPRotation = 0.15;
        double kIRotation = 0.01;
        double kDRotation = 0.02;
        rotationController = new PIDController(kPRotation, kIRotation, kDRotation);
        rotationController.setSetpoint(0.0);
        rotationController.setTolerance(0.5);
    }

    @Override
    public void execute() {
        boolean hasTargets = LimelightHelpers.getTV("");
        double targetArea = LimelightHelpers.getTA("");
        double horizontalOffset = LimelightHelpers.getTY("");

        // Check if the selected AprilTag ID is visible
        if (hasTargets && LimelightHelpers.getFiducialID("") == targetID) {
            double translationAdjustX = translationController.calculate(targetArea);
            double translationAdjustY = translationController.calculate(rotationController.getPositionError());
            double rotationAdjust = rotationController.calculate(-horizontalOffset);

            swerve.drive(new Translation2d(translationAdjustX, translationAdjustY), rotationAdjust, false);
        } else {
            // No AprilTag detected, stop
            swerve.drive(new Translation2d(0.0, 0.0), 0.0, false);
        }
    }
}