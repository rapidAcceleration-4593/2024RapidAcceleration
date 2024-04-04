// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.MatchConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.*;
import frc.robot.commands.AutoCommands.*;
import frc.robot.commands.BeakCommands.Intake.*;
import frc.robot.commands.BeakCommands.Shooter.*;
import frc.robot.commands.ClimberCommands.*;
import frc.robot.commands.PrimaryCommands.ManualControl.*;
import frc.robot.commands.PrimaryCommands.PresetPositions.*;
import frc.robot.commands.PrimaryCommands.Vision.VisionNeckAngleNotPressed;
import frc.robot.commands.PrimaryCommands.Vision.VisionNeckAnglePressed;
import frc.robot.commands.PrimaryCommands.Vision.VisionSwerveAlign;

import com.pathplanner.lib.auto.NamedCommands;
import java.io.File;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));

  private final BeakSubsystem beakSubsystem = new BeakSubsystem();
  private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();
  private final VisionSubsystem visionSubsystem = new VisionSubsystem(drivebase);
  private final NeckSubsystem neckSubsystem = new NeckSubsystem();

  private final CommandXboxController driverXbox = new CommandXboxController(0);
  private final CommandXboxController auxXbox = new CommandXboxController(1);

  private final String autoName = MatchConstants.autoName;

  public RobotContainer()
  {
    NamedCommands.registerCommand("IntakePosition", new IntakePosition(neckSubsystem));
    NamedCommands.registerCommand("SubwooferPosition", new SubwooferPosition(neckSubsystem));

    NamedCommands.registerCommand("Shooter", new ShooterAuto(beakSubsystem));
    NamedCommands.registerCommand("Intake", new IntakeAuto(beakSubsystem));
    NamedCommands.registerCommand("StopAll", new StopAllAuto(beakSubsystem));

    NamedCommands.registerCommand("ResetGyro", Commands.runOnce(drivebase::zeroGyro));

    configureBindings();

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the desired angle NOT angular rotation
    // Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
    //     () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
    //     () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
    //     () -> driverXbox.getRightX(),
    //     () -> driverXbox.getRightY());

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the angular velocity of the robot
    Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> driverXbox.getRightX()); // * 0.5

    // Command driveFieldOrientedDirectAngleSim = drivebase.simDriveCommand(
    //     () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
    //     () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
    //     () -> driverXbox.getRawAxis(2));

    drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    // drivebase.setDefaultCommand(
    //     !RobotBase.isSimulation() ? driveFieldOrientedDirectAngle : driveFieldOrientedDirectAngleSim);
  }

  private void configureBindings()
  {
    // Driver Controller
    driverXbox.back().onTrue(Commands.runOnce(drivebase::zeroGyro));
    driverXbox.start().whileTrue(new VisionSwerveAlign(visionSubsystem));

    driverXbox.y().whileTrue(new NeckUp(neckSubsystem));
    driverXbox.y().whileFalse(new NeckStop(neckSubsystem));
    driverXbox.a().whileTrue(new NeckDown(neckSubsystem));
    driverXbox.a().whileFalse(new NeckStop(neckSubsystem));

    // Auxilary Controller
    auxXbox.back().onTrue(new ManualControlEnabled(neckSubsystem));
    auxXbox.start().onTrue(new ManualControlDisabled(neckSubsystem));

    auxXbox.povDown().onTrue(new IntakePosition(neckSubsystem));
    auxXbox.povLeft().onTrue(new SubwooferPosition(neckSubsystem));
    auxXbox.povUp().onTrue(new AmpPosition(neckSubsystem));
    auxXbox.povRight().onTrue(new YeetPosition(neckSubsystem));
    auxXbox.b().whileTrue(new VisionNeckAnglePressed(neckSubsystem));
    auxXbox.b().whileFalse(new VisionNeckAngleNotPressed(neckSubsystem));

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
  }

  // Use this method to pass the autonomous command to the main class
  public Command getAutonomousCommand()
  {
    return drivebase.getAutonomousCommand(autoName);
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}