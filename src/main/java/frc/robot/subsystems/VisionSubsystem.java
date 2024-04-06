package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class VisionSubsystem extends SubsystemBase {

    private final SwerveSubsystem swerve;

    private final PIDController translationXController = new PIDController(1.0, 0.0, 0.0);
    private final PIDController translationYController = new PIDController(1.0, 0.0, 0.0);
    private final PIDController rotationController = new PIDController(0.1, 0.0, 0.0);
    private final PIDController rotationFlatController = new PIDController(1.0, 0.0, 0.0);

    private static double translationDeadband = 0.1;
    private static double rotationDeadband = 0.01;

    public VisionSubsystem(SwerveSubsystem swerve) {
        // Initialize Swerve
        this.swerve = swerve;
    }

    public void VisionSwerveAlign() {
        boolean hasTargets = LimelightHelpers.getTV("");
        Pose3d target = LimelightHelpers.getTargetPose3d_CameraSpace("");
        
        if (hasTargets) {
            if (LimelightHelpers.getFiducialID("") == 4 || LimelightHelpers.getFiducialID("") == 7) {
                double rotationAdjust = rotationController.calculate(LimelightHelpers.getTX(""), 0.0);

                if (Math.abs(rotationAdjust) < rotationDeadband) {
                    rotationAdjust = 0.0;
                }

                swerve.drive(new Translation2d(0.0, 0.0), -rotationAdjust, false);
            } else if ((LimelightHelpers.getFiducialID("") == 1) || (LimelightHelpers.getFiducialID("") == 12) || (LimelightHelpers.getFiducialID("") == 13) || (LimelightHelpers.getFiducialID("") == 14) || (LimelightHelpers.getFiducialID("") == 15) || (LimelightHelpers.getFiducialID("") == 16)) {
                double translationAdjustX = translationXController.calculate(target.getZ(), 6.0);
                double translationAdjustY = translationYController.calculate(target.getX());
                double rotationAdjust = rotationFlatController.calculate(target.getRotation().getY());
                
                if (Math.abs(rotationAdjust) > rotationDeadband) {
                    swerve.drive(new Translation2d(0.0, 0.0), -rotationAdjust, false);
                } else if (Math.abs(translationAdjustY) > translationDeadband) {
                    swerve.drive(new Translation2d(0.0, -translationAdjustY), 0.0, false);
                } else if (Math.abs(translationAdjustX) > translationDeadband) {
                    swerve.drive(new Translation2d(translationAdjustX, 0.0), 0.0, false);
                } else {
                    swerve.drive(new Translation2d(0.0, 0.0), 0.0, false);
                }
            } else {
                swerve.drive(new Translation2d(0.0, 0.0), 0.0, false);
            }
        }
    }
}
