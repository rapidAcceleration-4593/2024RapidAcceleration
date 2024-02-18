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
import frc.robot.subsystems.NeckSubsystem;
import frc.robot.subsystems.BeakSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.commands.BeakCommands.*;
import frc.robot.commands.NeckCommands.*;
import frc.robot.commands.NeckCommands.ManualControl.*;
import frc.robot.commands.NeckCommands.PresetPositions.AmpPosition;
import frc.robot.commands.NeckCommands.PresetPositions.IntakePosition;
import frc.robot.commands.NeckCommands.PresetPositions.SubwooferPosition;
import frc.robot.commands.VisionCommands.VisionSwerveAlign;

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
  public NeckSubsystem neckSubsystem = new NeckSubsystem();
  public BeakSubsystem beakSubsystem = new BeakSubsystem();
  public ClimberSubsystem climberSubsystem = new ClimberSubsystem();

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
    driverXbox.back().onTrue((new InstantCommand(() -> drivebase.zeroGyro()))); // Reset field orientation
    driverXbox.start().whileTrue(new VisionSwerveAlign(visionSubsystem)); // Align with AprilTag
    // driverXbox.b().whileTrue(new VisionNeckAngle(neckSubsystem));

    // Neck Rotation
    driverXbox.a().whileTrue(new NeckUp(neckSubsystem));
    driverXbox.a().whileFalse(new NeckHold(neckSubsystem));

    driverXbox.y().whileTrue(new SubwooferPosition(neckSubsystem));
    driverXbox.y().whileFalse(new NeckHold(neckSubsystem));

    driverXbox.b().whileTrue(new IntakePosition(neckSubsystem));
    driverXbox.b().whileFalse(new NeckHold(neckSubsystem));

    // driverXbox.b().whileTrue(new ClimberUp(climberSubsystem));
    // driverXbox.b().whileFalse(new ClimberStop(climberSubsystem));
    // driverXbox.start().whileTrue(new ClimberDown(climberSubsystem));
    // driverXbox.start().whileFalse(new ClimberStop(climberSubsystem));

    // Intake Commands
    driverXbox.rightBumper().whileTrue(new BeakIntake(beakSubsystem)); // Intake
    driverXbox.rightBumper().whileFalse(new BeakIntakeStop(beakSubsystem)); // Stop Intake
    driverXbox.leftBumper().whileTrue(new BeakOuttake(beakSubsystem)); // Outtake
    driverXbox.leftBumper().whileFalse(new BeakIntakeStop(beakSubsystem)); // Stop Outtake

    // Shooter Commands
    driverXbox.x().whileTrue(new BeakShooter(beakSubsystem));
    driverXbox.x().whileFalse(new BeakShooterStop(beakSubsystem));
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