package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {

    // Intake, Outtake, and Shooter
    private CANSparkMax intakeMotor;
    private CANSparkMax shooterTopMotor;
    private CANSparkMax shooterBottomMotor;

    private DigitalInput intakeLimitSwitch;
    private Timer shooterTimer;
    private boolean shooterTimerStarted;

    // Arm Rotation
    private CANSparkMax leftGearbox1;
    private CANSparkMax leftGearbox2;
    private CANSparkMax rightGearbox1;
    private CANSparkMax rightGearbox2;

    private CANSparkMax climberMotor;

    public ArmSubsystem() {
        // Initialize Motor Objects to CAN SparkMAX ID
        intakeMotor = new CANSparkMax(14, MotorType.kBrushless);
        shooterTopMotor = new CANSparkMax(13, MotorType.kBrushless);
        shooterBottomMotor = new CANSparkMax(15, MotorType.kBrushless);

        intakeLimitSwitch = new DigitalInput(0);
        shooterTimer = new Timer();
        shooterTimerStarted = false;

        leftGearbox1 = new CANSparkMax(19, MotorType.kBrushless);
        leftGearbox2 = new CANSparkMax(20, MotorType.kBrushless);
        rightGearbox1 = new CANSparkMax(8, MotorType.kBrushless);
        rightGearbox2 = new CANSparkMax(9, MotorType.kBrushless);

        climberMotor = new CANSparkMax(10, MotorType.kBrushless);
    }


    public void ArmUp() {
        ArmSetRotateSpeed(0.16);
    }

    public void ArmDown() {
        ArmSetRotateSpeed(-0.06);
    }

    public void ArmRotateStop() {
        ArmSetRotateSpeed(0.0);
    }

    public void Climber() {
        climberMotor.set(1.0);
    }

    public void ClimberReverse() {
        climberMotor.set(-1.0);
    }

    public void ClimberStop() {
        climberMotor.set(0.0);
    }




    public void ArmIntake() {
        if (intakeLimitSwitch.get()) {
            intakeMotor.set(-1.0);
        } else {
            intakeMotor.set(0.0);
        }
    }
    
    public void ArmOuttake() {
        intakeMotor.set(1.0);
    }

    public void ArmIntakeStop() {
        intakeMotor.set(0.0);
    }

    public void ArmShooter() {
        shooterTopMotor.set(0.7593);
        shooterBottomMotor.set(0.7593);

        if (!shooterTimerStarted) {
            shooterTimer.start();
            shooterTimerStarted = true;
        }

        if (shooterTimer.get() >= 0.8) {
            intakeMotor.set(-1.0);
        }
    }
    
    public void ArmShooterStop() {
        shooterTopMotor.set(0.0);
        shooterBottomMotor.set(0.0);
        intakeMotor.set(0.0);
        shooterTimer.stop();
        shooterTimer.reset();
        shooterTimerStarted = false;
    }

    private void ArmSetRotateSpeed(double speed) {
        leftGearbox1.set(speed);
        leftGearbox2.follow(leftGearbox1, false);
        rightGearbox1.follow(leftGearbox1, true);
        rightGearbox2.follow(leftGearbox1, true);
    }
}
