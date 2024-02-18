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

    private static double translationDeadband;
    private static double rotationDeadband;

    private static final int targetID = 4;

    public VisionSubsystem(SwerveSubsystem swerve) {
        // Initialize Swerve
        this.swerve = swerve;

        // Initalize TranslationX PID Controller
        translationControllerX = new PIDController(2.0, 0.0, 0.0);
        translationControllerX.setSetpoint(5);

        // Initalize TranslationY PID Controller
        translationControllerY = new PIDController(1.25, 0.0, 0.0);
        translationControllerY.setSetpoint(0.0);

        // Initalize Rotation PID Controller
        rotationController = new PIDController(4.0, 0.0, 0.0);
        rotationController.setSetpoint(0.0);

        translationDeadband = 0.1;
        rotationDeadband = 0.01;
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

            // Apply deadband for each direction
            if (Math.abs(translationAdjustX) < translationDeadband) {
                translationAdjustX = 0.0;
            }

            if (Math.abs(translationAdjustY) < translationDeadband) {
                translationAdjustY = 0.0;
            }

            if (Math.abs(rotationAdjust) < rotationDeadband) {
                rotationAdjust = 0.0;
            }

            swerve.drive(new Translation2d(translationAdjustX, translationAdjustY), -rotationAdjust, false);
        } else {
            // No AprilTag detected, stop
            swerve.drive(new Translation2d(0.0, 0.0), 0.0, false);
        }
    }
}
