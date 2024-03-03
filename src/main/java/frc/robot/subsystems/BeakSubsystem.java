package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
// import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.BeakConstants;
import frc.robot.Constants.NeckRotationConstants;

public class BeakSubsystem extends SubsystemBase {
    
    // Intake, Outtake, and Shooter
    private final CANSparkMax intakeMotor;
    private final CANSparkMax shooterTopMotor;
    private final CANSparkMax shooterBottomMotor;

    private final DigitalInput intakeLimitSwitch;
    private Timer shooterTimer;
    private boolean shooterTimerStarted;

    private final Encoder neckEncoder;

    public BeakSubsystem() {
        // Initialize Motor Objects to CAN SparkMAX ID
        intakeMotor = BeakConstants.intakeMotor;
        shooterTopMotor = BeakConstants.shooterTopMotor;
        shooterBottomMotor = BeakConstants.shooterBottomMotor;

        intakeLimitSwitch = BeakConstants.intakeLimitSwitch;
        shooterTimer = new Timer();
        shooterTimerStarted = false;

        neckEncoder = NeckRotationConstants.neckEncoder;
    }

    // Intake, Outtake, and Shooter
    public void BeakIntake() {
        intakeMotor.set(-1.0);
    }
    
    public void BeakOuttake() {
        intakeMotor.set(1.0);
    }

    public void BeakIntakeStop() {
        intakeMotor.set(0.0);
    }

    public void BeakShooter() {
        if (neckEncoder.get() >= 200) {
            shooterTopMotor.set(0.25);
            shooterBottomMotor.set(0.25);
        } else {
            shooterTopMotor.set(1.0);
            shooterBottomMotor.set(1.0);
        }

        if (!shooterTimerStarted) {
            shooterTimer.start();
            shooterTimerStarted = true;
        }

        if (shooterTimer.get() >= 0.8) {
            intakeMotor.set(-1.0);
        }
    }
    
    public void BeakShooterStop() {
        shooterTopMotor.set(0.0);
        shooterBottomMotor.set(0.0);
        intakeMotor.set(0.0);
        shooterTimer.stop();
        shooterTimer.reset();
        shooterTimerStarted = false;
    }

    public void ShooterAuto() {
        shooterTopMotor.set(0.75);
        shooterBottomMotor.set(0.75);
    }

    public void IntakeAuto() {
        intakeMotor.set(-1.0);
    }

    public void periodic() {
        if (!intakeLimitSwitch.get()) {
            BeakIntakeStop();
        }
    }
}
