// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.MatchConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.NeckSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.BeakSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.commands.AutoCommands.*;
import frc.robot.commands.BeakCommands.*;
import frc.robot.commands.ClimberCommands.*;
import frc.robot.commands.NeckCommands.*;
import frc.robot.commands.NeckCommands.PresetPositions.*;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import java.io.File;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{
  private final SwerveSubsystem drivebase;

  private final BeakSubsystem beakSubsystem;
  private final ClimberSubsystem climberSubsystem;
  private final VisionSubsystem visionSubsystem;
  private final NeckSubsystem neckSubsystem;

  private final CommandXboxController driverXbox;
  private final CommandXboxController auxXbox;

  private final String autoName;

  public RobotContainer()
  {
    drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));

    beakSubsystem = new BeakSubsystem();
    climberSubsystem = new ClimberSubsystem();
    visionSubsystem = new VisionSubsystem(drivebase);
    neckSubsystem = new NeckSubsystem();

    driverXbox = new CommandXboxController(0);
    auxXbox = new CommandXboxController(1);

    autoName = MatchConstants.autoName;

    NamedCommands.registerCommand("IntakePosition", new IntakePosition(neckSubsystem));
    NamedCommands.registerCommand("SubwooferPosition", new SubwooferPosition(neckSubsystem));
    NamedCommands.registerCommand("VisionAngle", new VisionNeckAngle(neckSubsystem));

    NamedCommands.registerCommand("Intake", new IntakeAuto(beakSubsystem));
    NamedCommands.registerCommand("Shooter", new ShooterAuto(beakSubsystem));
    NamedCommands.registerCommand("ShooterIntakeStop", new ShooterStopAuto(beakSubsystem));

    configureBindings();

    Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> driverXbox.getRightX());

        drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
  }

  private void configureBindings()
  {
    // Driver Controller
    driverXbox.back().onTrue((new InstantCommand(() -> drivebase.zeroGyro())));
    driverXbox.start().whileTrue(new VisionSwerveAlign(visionSubsystem));

    driverXbox.povUp().onTrue(new AmpPosition(neckSubsystem));
    driverXbox.povDown().onTrue(new IntakePosition(neckSubsystem));
    driverXbox.povLeft().onTrue(new SubwooferPosition(neckSubsystem));
    driverXbox.b().onTrue(new VisionNeckAngle(neckSubsystem));

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
  }

  // Use this method to pass the autonomous command to the main class
  public Command getAutonomousCommand()
  {
    return new PathPlannerAuto(autoName);
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}