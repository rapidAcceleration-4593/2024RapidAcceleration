package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class AlignWithAprilTag extends Command {
    
    private final SwerveSubsystem swerve;
    private final double kP = 0.09;
    private final double kI = 0.0;
    private final double kD = 0.0;
    private PIDController pid = new PIDController(kP, kI, kD);

    public AlignWithAprilTag(SwerveSubsystem swerve) {
        this.swerve = swerve;
    }

    @Override
    public void execute() {
        double verticalOffset = LimelightHelpers.getTY("");
        double targetArea = LimelightHelpers.getTA("");

        System.out.println(verticalOffset);

        // if (verticalOffset < -2 || verticalOffset > 2) {
        //     swerve.drive(new Translation2d(0.0, -verticalOffset * 0.09), -verticalOffset * 0.09, false);
        // } else {
        //     swerve.drive(new Translation2d(0.0, 0.0), 0.0, false);
        // }

        if (targetArea < 2 && targetArea > 0) {
            swerve.drive(new Translation2d(-1, 0.0), 0.0, false);
        } else {
            swerve.drive(new Translation2d(0.0, 0.0), 0.0, false);
        
             if (verticalOffset < -2 || verticalOffset > 2) {
                swerve.drive(new Translation2d(0.0, -verticalOffset * 0.09), -verticalOffset * 0.09, false);
            } else {
                swerve.drive(new Translation2d(0.0, 0.0), 0.0, false);
        }
        }
    }
}
