package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.BeakConstants;
import frc.robot.Constants.NeckRotationConstants;

public class BeakSubsystem extends SubsystemBase {
    
    private final CANSparkMax intakeMotor;
    private final CANSparkMax shooterTopMotor;
    private final CANSparkMax shooterBottomMotor;

    private final DigitalInput intakeLimitSwitch;
    private final Encoder neckEncoder;

    private Timer shooterTimer;
    private boolean shooterTimerStarted;

    public BeakSubsystem() {
        intakeMotor = BeakConstants.intakeMotor;
        shooterTopMotor = BeakConstants.shooterTopMotor;
        shooterBottomMotor = BeakConstants.shooterBottomMotor;

        intakeLimitSwitch = BeakConstants.intakeLimitSwitch;
        neckEncoder = NeckRotationConstants.neckEncoder;

        shooterTimer = new Timer();
        shooterTimerStarted = false;
    }

    public void BeakIntake() {
        if (intakeLimitSwitch.get()) {
            intakeMotor.set(-1.0);
        } else {
            BeakIntakeStop();
        }
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
        shooterTopMotor.set(1.0);
        shooterBottomMotor.set(1.0);
    }

    public void IntakeAuto() {
        intakeMotor.set(-1.0);
    }
}
