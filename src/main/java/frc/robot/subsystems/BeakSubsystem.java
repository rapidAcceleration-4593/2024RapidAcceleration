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

    // private final PIDController shooterSpeedController;

    public BeakSubsystem() {
        // Initialize Motor Objects to CAN SparkMAX ID
        intakeMotor = BeakConstants.intakeMotor;
        shooterTopMotor = BeakConstants.shooterTopMotor;
        shooterBottomMotor = BeakConstants.shooterBottomMotor;

        intakeLimitSwitch = BeakConstants.intakeLimitSwitch;
        shooterTimer = new Timer();
        shooterTimerStarted = false;

        neckEncoder = NeckRotationConstants.neckEncoder;

        // shooterSpeedController = new PIDController(0.525, 0.7, 0.035);
    }

    // Intake, Outtake, and Shooter
    public void BeakIntake() {
        if (intakeLimitSwitch.get()) {
            intakeMotor.set(-1.0);
        } else {
            // intakeMotor.set(0.0);
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
        // double desiredVelocity = 3.25;
        double currentVelocity = shooterTopMotor.getEncoder().getVelocity();

        // double adjustedSpeed = shooterSpeedController.calculate(currentVelocity, desiredVelocity);

        System.out.println("<-------------------->");
        System.out.println("Shooter Velocity: " + currentVelocity);

        if (neckEncoder.get() >= 200) {
            shooterTopMotor.set(0.25);
            shooterBottomMotor.set(0.25);
        } else {
            shooterTopMotor.set(0.75);
            shooterBottomMotor.set(0.75);
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
}
