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
    private final PIDController translationControllerX;
    private final PIDController translationControllerY;

    private static final int targetID = 11;

    public VisionCommand(SwerveSubsystem swerve) {
        // Initialize Swerv3
        this.swerve = swerve;

        // Initalize TranslationX PID Controller
        double kPTranslationX = 1.8;
        double kITranslationX = 0.03;
        double KDTranslationX = 0.06;
        translationControllerX = new PIDController(kPTranslationX, kITranslationX, KDTranslationX);
        translationControllerX.setSetpoint(1.0);
        translationControllerX.setTolerance(0.25);

        // Initalize TranslationY PID Controller
        double kPTranslationY = 0.06;
        double kITranslationY = 0.0;
        double KDTranslationY = 0.01;
        translationControllerY = new PIDController(kPTranslationY, kITranslationY, KDTranslationY);
        translationControllerY.setSetpoint(0.0);
        translationControllerY.setTolerance(0.6);

        // Initialize Rotation PID Controller
        double kPRotation = 0.14;
        double kIRotation = 0.001;
        double kDRotation = 0.01;
        rotationController = new PIDController(kPRotation, kIRotation, kDRotation);
        rotationController.setSetpoint(0.0);
        rotationController.setTolerance(0.7);
    }

    @Override
    public void execute() {
        boolean hasTargets = LimelightHelpers.getTV("");
        double targetArea = LimelightHelpers.getTA("");
        double horizontalOffset = LimelightHelpers.getTY("");

        // Check if the selected AprilTag ID is visible
        if (hasTargets && LimelightHelpers.getFiducialID("") == targetID) {
            double translationAdjustX = translationControllerX.calculate(targetArea);
            double translationAdjustY = translationControllerY.calculate(rotationController.getPositionError());
            double rotationAdjust = rotationController.calculate(horizontalOffset);

            swerve.drive(new Translation2d(-translationAdjustX, -translationAdjustY), rotationAdjust, false);
        } else {

            // No AprilTag detected, stop
            swerve.drive(new Translation2d(0.0, 0.0), 0.0, false);
        }
        
    }
}