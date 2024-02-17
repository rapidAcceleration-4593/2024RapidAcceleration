package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class VisionSubsystem extends SubsystemBase {

    private final SwerveSubsystem swerve;
    private final PIDController translationControllerX;
    private final PIDController translationControllerY;
    private final PIDController rotationController;

    private static final int targetID = 4;

    public VisionSubsystem(SwerveSubsystem swerve) {
        // Initialize Swerve
        this.swerve = swerve;

        // Initalize TranslationX PID Controller
        double kPTranslationX = 2.0;
        double kITranslationX = 0.0; // Adjust accordingly
        double KDTranslationX = 0.0;
        translationControllerX = new PIDController(kPTranslationX, kITranslationX, KDTranslationX);
        translationControllerX.setSetpoint(5); // 5 Meters
        translationControllerX.setTolerance(0.0);

        // Initalize TranslationY PID Controller
        double kPTranslationY = 1.25;
        double kITranslationY = 0.0; // Adjust accordingly
        double KDTranslationY = 0.0;
        translationControllerY = new PIDController(kPTranslationY, kITranslationY, KDTranslationY);
        translationControllerY.setSetpoint(0.0);
        translationControllerY.setTolerance(0.1);

        // Initalize Rotation PID Controller
        double kPRotation = 4.0;
        double kIRotation = 0.0; // Adjust accordingly
        double kDRotation = 0.001;
        rotationController = new PIDController(kPRotation, kIRotation, kDRotation);
        rotationController.setSetpoint(0.0);
        rotationController.setTolerance(0.002);
    }

    public void VisionSwerveAlign() {
        boolean hasTargets = LimelightHelpers.getTV("");

        // Check if the selected AprilTag ID is visible
        if (hasTargets && LimelightHelpers.getFiducialID("") == targetID) {
            Pose3d target = LimelightHelpers.getTargetPose3d_CameraSpace("");

            double translationAdjustX = translationControllerX.calculate(target.getZ());
            double translationAdjustY = translationControllerY.calculate(target.getX());
            double rotationAdjust = rotationController.calculate(target.getRotation().getY());

            System.out.println("<-------------------->");
            System.out.println("Z Distance: " + target.getZ());
            System.out.println("X Distance: " + target.getX());
            System.out.println("Y Rotation: " + target.getRotation().getY());

            swerve.drive(new Translation2d(translationAdjustX, 0.0), 0.0, false);

            // if (target.getX() > 0.1 && target.getX() < -0.1) {
            //     swerve.drive(new Translation2d(0.0, translationAdjustY), 0.0, false);
            // }

            // if (target.getRotation().getY() > 0.01 && target.getRotation().getY() < -0.01) {
            //     swerve.drive(new Translation2d(0.0, 0.0), -rotationAdjust, false);
            // }
        } else {
            // No AprilTag detected, stop
            swerve.drive(new Translation2d(0.0, 0.0), 0.0, false);
        }
    }
}
