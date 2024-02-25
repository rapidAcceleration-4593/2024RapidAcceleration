// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import swervelib.math.Matter;
import swervelib.parser.PIDFConfig;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants.
 * This class should not be used for any other purpose. All constants should be declared globally (i.e. public static).
 * Do not put anything functional in this class.
 */
public final class Constants
{
  public static final double ROBOT_MASS = (100) * 0.453592; // 90 lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag

  public static final class NeckRotationConstants {
    public final static CANSparkMax leftGearbox1 = new CANSparkMax(19, MotorType.kBrushless);
    public final static CANSparkMax leftGearbox2 = new CANSparkMax(20, MotorType.kBrushless);
    public final static CANSparkMax rightGearbox1 = new CANSparkMax(8, MotorType.kBrushless);
    public final static CANSparkMax rightGearbox2 = new CANSparkMax(9, MotorType.kBrushless);
    // public final static DigitalInput topLimitSwitch = new DigitalInput(0);
    public final static DigitalInput bottomLimitSwitch = new DigitalInput(1);
    public final static Encoder neckEncoder = new Encoder(8, 9);
  }

  public static final class NeckExtensionConstants {
    public final static CANSparkMax neckExtensionMotor = new CANSparkMax(23, MotorType.kBrushless);
    // public final static DigitalInput extensionTopLimitSwitch = new DigitalInput(5);
    public final static DigitalInput extensionBottomLimitSwitch = new DigitalInput(2);
    public final static DutyCycleEncoder throughBoreEncoder = new DutyCycleEncoder(3);
  }

  public static final class BeakConstants {
    public final static CANSparkMax intakeMotor = new CANSparkMax(15, MotorType.kBrushless);
    public final static CANSparkMax shooterTopMotor = new CANSparkMax(13, MotorType.kBrushless);
    public final static CANSparkMax shooterBottomMotor = new CANSparkMax(14, MotorType.kBrushless);
    public final static DigitalInput intakeLimitSwitch = new DigitalInput(0);
  }

  public static final class ClimberConstants {
    public final static CANSparkMax climberMotor = new CANSparkMax(10, MotorType.kBrushless);
  }

  public static final class Auton
  {
    public static final PIDFConfig TranslationPID = new PIDFConfig(0.7, 0, 0.0);
    public static final PIDFConfig angleAutoPID   = new PIDFConfig(0.4, 0, 0.01);

    public static final double MAX_ACCELERATION = 2;
  }

  public static final class Drivebase
  {
    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants
  {
    // Joystick Deadband
    public static final double LEFT_X_DEADBAND  = 0.15;
    public static final double LEFT_Y_DEADBAND  = 0.15;
    public static final double RIGHT_X_DEADBAND = 0.15;
    public static final double TURN_CONSTANT    = 6;
  }
}
