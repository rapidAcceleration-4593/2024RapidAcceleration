package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class VisionCommand extends Command {

    private final SwerveSubsystem swerve;
    private final PIDController rotationController;

    public VisionCommand(SwerveSubsystem swerve) {
        this.swerve = swerve;

        double kPRotation = 0.15;
        double kIRotation = 0.01;
        double kDRotation = 0.02;
        rotationController = new PIDController(kPRotation, kIRotation, kDRotation);
        rotationController.setSetpoint(0.0);
        // rotationController.setTolerance(3);
    }

    @Override
    public void execute() {
        double horizontalOffset = LimelightHelpers.getTY("");
        double targetArea = LimelightHelpers.getTA("");

        double kPTranslation = 1.0;
        double kPRotation = 0.09;
        
        double rotateAdjust = rotationController.calculate(horizontalOffset);

        swerve.drive(new Translation2d(0.0, 0.0), rotateAdjust, false);

        // if (targetArea < 1.5 && targetArea > 0) {
        //     swerve.drive(new Translation2d(-1 * kPTranslation, 0.0), 0.0, false);
        // } else if (targetArea > 3) {
        //     swerve.drive(new Translation2d(1 * kPTranslation, 0.0), 0.0, false);
        // } else {
        //     if (horizontalOffset <= -2 || horizontalOffset >= 2) {
        //         swerve.drive(new Translation2d(0.0, -horizontalOffset * kPRotation), -horizontalOffset * kPRotation, false);
        //     } else {
        //         swerve.drive(new Translation2d(0.0, 0.0), 0.0, false);
        //     }
        // }
    }
}