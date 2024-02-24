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
import frc.robot.commands.AutoCommands.ShooterAuto;
import frc.robot.commands.BeakCommands.*;
// import frc.robot.commands.ClimberCommands.*;
import frc.robot.commands.NeckRotationCommands.*;
// import frc.robot.commands.NeckExtensionCommands.*;
import frc.robot.commands.NeckRotationCommands.ManualControl.*;
import frc.robot.commands.NeckRotationCommands.PresetPositions.*;

import java.io.File;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{
  // Define the robot's subsystems and commands
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));

  private final BeakSubsystem beakSubsystem;
  private final ClimberSubsystem climberSubsystem;
  private final VisionSubsystem visionSubsystem;
  private final NeckRotationSubsystem neckRotationSubsystem;
  private final NeckExtensionSubsystem neckExtensionSubsystem;

  private final CommandXboxController driverXbox;
  private final CommandXboxController auxXbox;

  // The container for the robot. Contains subsystems, OI devices, and commands.
  public RobotContainer()
  {
    beakSubsystem = new BeakSubsystem();
    climberSubsystem = new ClimberSubsystem();
    visionSubsystem = new VisionSubsystem(drivebase);
    neckRotationSubsystem = new NeckRotationSubsystem();
    neckExtensionSubsystem = new NeckExtensionSubsystem();

    driverXbox = new CommandXboxController(0);
    auxXbox = new CommandXboxController(1);

    NamedCommands.registerCommand("SubwooferPosition", new SubwooferPosition(neckRotationSubsystem));
    NamedCommands.registerCommand("Follower", new NeckFollower(neckRotationSubsystem));
    NamedCommands.registerCommand("IntakePosition", new IntakePosition(neckRotationSubsystem));
    NamedCommands.registerCommand("Shooter", new ShooterAuto(beakSubsystem));
    NamedCommands.registerCommand("ShooterStop", new BeakShooterStop(beakSubsystem));
    NamedCommands.registerCommand("Intake", new BeakIntake(beakSubsystem));
    NamedCommands.registerCommand("IntakeStop", new BeakIntakeStop(beakSubsystem));

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
    driverXbox.back().onTrue((new InstantCommand(() -> drivebase.zeroGyro())));
    driverXbox.start().whileTrue(new VisionSwerveAlign(visionSubsystem));

    driverXbox.povUp().whileTrue(new NeckUp(neckRotationSubsystem));
    driverXbox.povUp().whileFalse(new NeckFollower(neckRotationSubsystem));

    driverXbox.povDown().whileTrue(new NeckDown(neckRotationSubsystem));
    driverXbox.povDown().whileFalse(new NeckFollower(neckRotationSubsystem));

    driverXbox.povLeft().onTrue(new SubwooferPosition(neckRotationSubsystem));
    driverXbox.povLeft().whileFalse(new NeckFollower(neckRotationSubsystem));

    driverXbox.povRight().onTrue(new IntakePosition(neckRotationSubsystem));
    driverXbox.povRight().whileFalse(new NeckFollower(neckRotationSubsystem));

    driverXbox.y().onTrue(new AmpPosition(neckRotationSubsystem));
    driverXbox.y().whileFalse(new NeckFollower(neckRotationSubsystem));

    driverXbox.b().onTrue(new VisionNeckAngle(neckRotationSubsystem));
    driverXbox.b().whileFalse(new NeckFollower(neckRotationSubsystem));

    driverXbox.rightBumper().whileTrue(new BeakIntake(beakSubsystem));
    driverXbox.rightBumper().whileFalse(new BeakIntakeStop(beakSubsystem));

    driverXbox.leftBumper().whileTrue(new BeakOuttake(beakSubsystem));
    driverXbox.leftBumper().whileFalse(new BeakIntakeStop(beakSubsystem));

    driverXbox.x().whileTrue(new BeakShooter(beakSubsystem));
    driverXbox.x().whileFalse(new BeakShooterStop(beakSubsystem));


    // auxXbox.define().whileTrue(new ExtensionIn(neckExtensionSubsystem));
    // auxXbox.define().whileFalse(new ExtensionStop(neckExtensionSubsystem));

    // auxXbox.define().whileTrue(new ExtensionOut(neckExtensionSubsystem));
    // auxXbox.define().whileFalse(new ExtensionStop(neckExtensionSubsystem));

    // driverXbox.define().whileTrue(new ClimberUp(climberSubsystem));
    // driverXbox.define().whileFalse(new ClimberStop(climberSubsystem));
    
    // driverXbox.define().whileTrue(new ClimberDown(climberSubsystem));
    // driverXbox.define().whileFalse(new ClimberStop(climberSubsystem));
  }

  // Use this method to pass the autonomous command to the main class
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    // return drivebase.getAutonomousCommand("Forward", true);
    return new PathPlannerAuto("FirstAuto");
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}