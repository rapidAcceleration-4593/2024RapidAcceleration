package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BeakSubsystem extends SubsystemBase {
    
    // Intake, Outtake, and Shooter
    private final CANSparkMax intakeMotor;
    private final CANSparkMax shooterTopMotor;
    private final CANSparkMax shooterBottomMotor;

    private DigitalInput intakeLimitSwitch;
    private Timer shooterTimer;
    private boolean shooterTimerStarted;

    private final PIDController shooterSpeedController;

    public BeakSubsystem() {
        // Initialize Motor Objects to CAN SparkMAX ID
        intakeMotor = new CANSparkMax(14, MotorType.kBrushless);
        shooterTopMotor = new CANSparkMax(13, MotorType.kBrushless);
        shooterBottomMotor = new CANSparkMax(15, MotorType.kBrushless);

        intakeLimitSwitch = new DigitalInput(0);
        shooterTimer = new Timer();
        shooterTimerStarted = false;

        shooterSpeedController = new PIDController(0.1, 0.0, 0.0);
    }

    // Intake, Outtake, and Shooter
    public void BeakIntake() {
        if (intakeLimitSwitch.get()) {
            intakeMotor.set(-1.0);
        } else {
            intakeMotor.set(0.0);
        }
    }
    
    public void BeakOuttake() {
        intakeMotor.set(1.0);
    }

    public void BeakIntakeStop() {
        intakeMotor.set(0.0);
    }

    public void BeakShooter() {
        double desiredVelocity = 3000;
        double currentVelocity = shooterTopMotor.getEncoder().getVelocity();

        double adjustedSpeed = shooterSpeedController.calculate(currentVelocity, desiredVelocity);

        System.out.println("<-------------------->");
        System.out.println("Shooter Velocity: " + currentVelocity);

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
    
    public void BeakShooterStop() {
        shooterTopMotor.set(0.0);
        shooterBottomMotor.set(0.0);
        intakeMotor.set(0.0);
        shooterTimer.stop();
        shooterTimer.reset();
        shooterTimerStarted = false;
    }
}
