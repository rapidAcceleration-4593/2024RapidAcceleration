package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class VisionCommand extends Command {
    
    private final SwerveSubsystem swerve;
    private final double kPTranslation = 1.0;
    private final double kPRotation = 0.09;

    public VisionCommand(SwerveSubsystem swerve) {
        this.swerve = swerve;
    }

    @Override
    public void execute() {
        double verticalOffset = LimelightHelpers.getTY("");
        double targetArea = LimelightHelpers.getTA("");

        System.out.println(verticalOffset);

        if (targetArea <= 2 && targetArea > 0) {
            swerve.drive(new Translation2d(-1 * kPTranslation, 0.0), 0.0, false);
        } else if (targetArea >= 5) {
            swerve.drive(new Translation2d(-1 * kPTranslation, 0.0), 0.0, false);
        } else {
            if (verticalOffset <= -2 || verticalOffset >= 2) {
                swerve.drive(new Translation2d(0.0, -verticalOffset * kPRotation), -verticalOffset * kPRotation, false);
            } else {
                swerve.drive(new Translation2d(0.0, 0.0), 0.0, false);
            }
        }
    }
}
