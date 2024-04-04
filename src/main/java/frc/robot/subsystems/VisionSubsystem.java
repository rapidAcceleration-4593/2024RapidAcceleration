package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class VisionSubsystem extends SubsystemBase {

    private final SwerveSubsystem swerve;

    private final PIDController rotationController = new PIDController(0.1, 0.0, 0.0);

    private static double rotationDeadband = 0.01;

    public VisionSubsystem(SwerveSubsystem swerve) {
        // Initialize Swerve
        this.swerve = swerve;
    }

    public void VisionSwerveAlign() {
        boolean hasTargets = LimelightHelpers.getTV("");
        
        if (hasTargets && (LimelightHelpers.getFiducialID("") == 4 || LimelightHelpers.getFiducialID("") == 7)) {
            double rotationAdjust = rotationController.calculate(LimelightHelpers.getTX(""), 0.0);

            if (Math.abs(rotationAdjust) < rotationDeadband) {
                rotationAdjust = 0.0;
            }

            swerve.drive(new Translation2d(0.0, 0.0), -rotationAdjust, false);
        } else {
            swerve.drive(new Translation2d(0.0, 0.0), 0.0, false);
        }
    }
}
