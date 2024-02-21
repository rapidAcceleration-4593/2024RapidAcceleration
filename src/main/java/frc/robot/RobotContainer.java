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
import frc.robot.subsystems.NeckRotationSubsystem;
import frc.robot.subsystems.BeakSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.NeckExtensionSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.commands.BeakCommands.*;
// import frc.robot.commands.ClimberCommands.*;
import frc.robot.commands.NeckRotationCommands.*;
// import frc.robot.commands.NeckExtensionCommands.*;
import frc.robot.commands.NeckRotationCommands.ManualControl.*;
import frc.robot.commands.NeckRotationCommands.PresetPositions.*;
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

  public BeakSubsystem beakSubsystem = new BeakSubsystem();
  public ClimberSubsystem climberSubsystem = new ClimberSubsystem();
  public VisionSubsystem visionSubsystem = new VisionSubsystem(drivebase);
  public NeckRotationSubsystem neckRotationSubsystem = new NeckRotationSubsystem();
  public NeckExtensionSubsystem neckExtensionSubsystem = new NeckExtensionSubsystem();

  CommandXboxController driverXbox = new CommandXboxController(0);
  CommandXboxController auxXbox = new CommandXboxController(1);

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
    // Driver Controller
    driverXbox.back().onTrue((new InstantCommand(() -> drivebase.zeroGyro()))); // Reset field orientation
    driverXbox.start().whileTrue(new VisionSwerveAlign(visionSubsystem)); // Align with AprilTag

    driverXbox.y().whileTrue(new NeckUp(neckRotationSubsystem));
    driverXbox.y().whileFalse(new NeckFollower(neckRotationSubsystem));

    driverXbox.a().whileTrue(new NeckDown(neckRotationSubsystem));
    driverXbox.a().whileFalse(new NeckFollower(neckRotationSubsystem));

    driverXbox.b().whileTrue(new SubwooferPosition(neckRotationSubsystem));
    driverXbox.b().whileFalse(new NeckFollower(neckRotationSubsystem));

    driverXbox.x().whileTrue(new IntakePosition(neckRotationSubsystem));
    driverXbox.x().whileFalse(new NeckFollower(neckRotationSubsystem));

    // driverXbox.define().whileTrue(new VisionNeckAngle(neckRotationSubsystem));

    // driverXbox.define().whileTrue(new ClimberUp(climberSubsystem));
    // driverXbox.define().whileFalse(new ClimberStop(climberSubsystem));
    
    // driverXbox.define().whileTrue(new ClimberDown(climberSubsystem));
    // driverXbox.define().whileFalse(new ClimberStop(climberSubsystem));



    // Auxiliary Controller
    auxXbox.rightBumper().whileTrue(new BeakIntake(beakSubsystem)); // Start Intake
    auxXbox.rightBumper().whileFalse(new BeakIntakeStop(beakSubsystem)); // Stop Intake

    auxXbox.leftBumper().whileTrue(new BeakOuttake(beakSubsystem)); // Start Outtake
    auxXbox.leftBumper().whileFalse(new BeakIntakeStop(beakSubsystem)); // Stop Outtake

    auxXbox.x().whileTrue(new BeakShooter(beakSubsystem)); // Start Shooter
    auxXbox.x().whileFalse(new BeakShooterStop(beakSubsystem)); // Stop Shooter

    auxXbox.y().whileTrue(new AmpPosition(neckRotationSubsystem)); // Start Shooter
    auxXbox.y().whileFalse(new NeckFollower(neckRotationSubsystem)); // Stop Shooter

    // auxXbox.define().whileTrue(new ExtensionIn(neckExtensionSubsystem));
    // auxXbox.define().whileFalse(new ExtensionStop(neckExtensionSubsystem));

    // auxXbox.define().whileTrue(new ExtensionOut(neckExtensionSubsystem));
    // auxXbox.define().whileFalse(new ExtensionStop(neckExtensionSubsystem));
  }

  // Use this method to pass the autonomous command to the main class
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    return drivebase.getAutonomousCommand("Forward", true);
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}