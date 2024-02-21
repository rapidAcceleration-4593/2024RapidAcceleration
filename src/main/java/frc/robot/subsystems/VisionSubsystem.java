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

        translationControllerX = new PIDController(0.7, 0.0, 0.0);
        translationControllerY = new PIDController(0.7, 0.0, 0.0);
        rotationController = new PIDController(0.4, 0.0, 0.01);

        translationDeadband = 0.1;
        rotationDeadband = 0.01;
    }

    public void VisionSwerveAlign() {
        boolean hasTargets = LimelightHelpers.getTV("");
        
        if (hasTargets && LimelightHelpers.getFiducialID("") == targetID) {
            Pose3d target = LimelightHelpers.getTargetPose3d_CameraSpace("");

            double translationAdjustX = translationControllerX.calculate(target.getZ(), 5.0);
            double translationAdjustY = translationControllerY.calculate(target.getX(), 0.0);
            double rotationAdjust = rotationController.calculate(target.getRotation().getY(), 0.0);

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

            // swerve.drive(new Translation2d(translationAdjustX, translationAdjustY), -rotationAdjust, false);
        } else {
            swerve.drive(new Translation2d(0.0, 0.0), 0.0, false);
        }
    }
}
