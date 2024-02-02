package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class VisionCommand extends Command {

    private final SwerveSubsystem swerve;
    private final PIDController translationControllerX;
    private final PIDController translationControllerY;
    private final PIDController rotationController;

    private static final int targetID = 1;

    public VisionCommand(SwerveSubsystem swerve) {
        // Initialize Swerve
        this.swerve = swerve;

        // Initalize TranslationX PID Controller
        double kPTranslationX = 2.0;
        double kITranslationX = 0.0;
        double KDTranslationX = 0.01;
        translationControllerX = new PIDController(kPTranslationX, kITranslationX, KDTranslationX);
        translationControllerX.setSetpoint(2.0); // 2 meter
        translationControllerX.setTolerance(0.1);

        // Initalize TranslationY PID Controller
        double kPTranslationY = 1.0;
        double kITranslationY = 0.0;
        double KDTranslationY = 0.01;
        translationControllerY = new PIDController(kPTranslationY, kITranslationY, KDTranslationY);
        translationControllerY.setSetpoint(0.0);
        translationControllerY.setTolerance(0.1);

        // Initalize Rotation PID Controller
        double kPRotation = 0.2;
        double kIRotation = 0.0;
        double kDRotation = 0.01;
        rotationController = new PIDController(kPRotation, kIRotation, kDRotation);
        rotationController.setSetpoint(0.0);
        rotationController.setTolerance(0.1);
    }

    @Override
    public void execute() {
        boolean hasTargets = LimelightHelpers.getTV("");

        // Check if the selected AprilTag ID is visible
        if (hasTargets && LimelightHelpers.getFiducialID("") == targetID) {
            Pose3d target = LimelightHelpers.getTargetPose3d_CameraSpace("");

            double translationAdjustX = translationControllerX.calculate(target.getZ());
            double translationAdjustY = translationControllerY.calculate(target.getX());
            double rotationAdjust = rotationController.calculate(LimelightHelpers.getTY(""));

            swerve.drive(new Translation2d(translationAdjustX, translationAdjustY), rotationAdjust, false);
        } else {
            // No AprilTag detected, stop
            swerve.drive(new Translation2d(0.0, 0.0), 0.0, false);
        }
    }
}