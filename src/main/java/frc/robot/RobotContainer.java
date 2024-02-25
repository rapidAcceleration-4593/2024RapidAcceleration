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
import frc.robot.commands.AutoCommands.ShooterAuto;
import frc.robot.commands.BeakCommands.*;
import frc.robot.commands.ClimberCommands.*;
import frc.robot.commands.ExtensionCommands.*;
import frc.robot.commands.NeckCommands.*;
import frc.robot.commands.NeckCommands.ManualControl.NeckUp;
import frc.robot.commands.NeckCommands.PresetPositions.*;

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
  private final NeckSubsystem neckSubsystem;

  private final CommandXboxController driverXbox;
  private final CommandXboxController auxXbox;

  // The container for the robot. Contains subsystems, OI devices, and commands.
  public RobotContainer()
  {
    beakSubsystem = new BeakSubsystem();
    climberSubsystem = new ClimberSubsystem();
    visionSubsystem = new VisionSubsystem(drivebase);
    neckSubsystem = new NeckSubsystem();

    driverXbox = new CommandXboxController(0);
    auxXbox = new CommandXboxController(1);

    NamedCommands.registerCommand("SubwooferPosition", new SubwooferPosition(neckSubsystem));
    NamedCommands.registerCommand("Follower", new NeckFollower(neckSubsystem));
    NamedCommands.registerCommand("IntakePosition", new IntakePosition(neckSubsystem));
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

    driverXbox.povUp().onTrue(new AmpPosition(neckSubsystem));
    driverXbox.povUp().whileFalse(new NeckFollower(neckSubsystem));

    driverXbox.povDown().onTrue(new IntakePosition(neckSubsystem));
    driverXbox.povDown().whileFalse(new NeckFollower(neckSubsystem));

    driverXbox.povLeft().onTrue(new SubwooferPosition(neckSubsystem));
    driverXbox.povLeft().whileFalse(new NeckFollower(neckSubsystem));


    driverXbox.povRight().onTrue(new DrivingPosition(neckSubsystem));
    driverXbox.povRight().whileFalse(new NeckFollower(neckSubsystem));

    driverXbox.b().onTrue(new VisionNeckAngle(neckSubsystem));
    driverXbox.b().whileFalse(new NeckFollower(neckSubsystem));

    // driverXbox.a().whileTrue(new NeckUp(neckSubsystem));


    // Auxilary Controller
    auxXbox.rightBumper().whileTrue(new BeakShooter(beakSubsystem));
    auxXbox.rightBumper().whileFalse(new BeakShooterStop(beakSubsystem));

    auxXbox.leftBumper().whileTrue(new BeakIntake(beakSubsystem));
    auxXbox.leftBumper().whileFalse(new BeakIntakeStop(beakSubsystem));

    auxXbox.x().whileTrue(new BeakOuttake(beakSubsystem));
    auxXbox.x().whileFalse(new BeakIntakeStop(beakSubsystem));

    auxXbox.povUp().whileTrue(new ClimberUp(climberSubsystem));
    auxXbox.povUp().whileFalse(new ClimberStop(climberSubsystem));
    
    auxXbox.povDown().whileTrue(new ClimberDown(climberSubsystem));
    auxXbox.povDown().whileFalse(new ClimberStop(climberSubsystem));

    // auxXbox.y().whileTrue(new ExtendOut(neckSubsystem));
    // auxXbox.y().whileFalse(new ExtendStop(neckSubsystem));

    // auxXbox.a().whileTrue(new ExtendIn(neckSubsystem));
    // auxXbox.a().whileFalse(new ExtendStop(neckSubsystem));
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