// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.VisionAlignCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.commands.ArmCommands.*;
import frc.robot.commands.ArmCommands.ManualControl.*;
import java.io.File;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{
  // Define the robot's subsystems and commands
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
  public VisionSubsystem visionSubsystem = new VisionSubsystem(drivebase);
  public ArmSubsystem armSubsystem = new ArmSubsystem();

  CommandXboxController driverXbox = new CommandXboxController(0);

  // The container for the robot. Contains subsystems, OI devices, and commands.
  public RobotContainer()
  {
    // Configure the trigger bindings
    configureBindings();

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the angular velocity of the robot

    Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> driverXbox.getRightX());

        drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
  }

  
  // Use this method to define your trigger->command mappings
  private void configureBindings()
  {
    // Driver Buttons
    // driverXbox.back().onTrue((new InstantCommand(() -> drivebase.zeroGyro()))); // Reset field orientation
    // driverXbox.start().whileTrue(new VisionAlignCommand(visionSubsystem)); // Align with AprilTag

    // Rotating Arm Commands
    // driverXbox.y().whileTrue(new ArmRotateUp(armSubsystem)); // Raise Arm
    // driverXbox.y().whileFalse(new ArmRotateStop(armSubsystem)); // Stop Arm
    // driverXbox.a().whileTrue(new ArmRotateDown(armSubsystem)); // Lower Arm
    // driverXbox.a().whileFalse(new ArmRotateStop(armSubsystem)); // Stop Arm

    // Intake Commands
    driverXbox.rightBumper().whileTrue(new ArmIntake(armSubsystem)); // Intake
    driverXbox.leftBumper().whileTrue(new ArmOuttake(armSubsystem)); // Outtake

    driverXbox.rightBumper().whileFalse(new ArmIntakeStop(armSubsystem)); // Stop Intake & Outtake
    driverXbox.leftBumper().whileFalse(new ArmIntakeStop(armSubsystem)); // Stop Intake & Outtake

    // Shooter Commands
    driverXbox.x().whileTrue(new ArmShooter(armSubsystem));
    driverXbox.x().whileFalse(new ArmShooterStop(armSubsystem));
  }

  // Use this method to pass the autonomous command to the main class
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    return drivebase.getAutonomousCommand("New Path", true);
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}