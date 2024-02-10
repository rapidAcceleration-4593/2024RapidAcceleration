package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {

    // Define Empty Arm Motor Objects
    private CANSparkMax intakeMotor;
    private CANSparkMax shooterTopMotor;
    private CANSparkMax shooterBottomMotor;

    // Limit Switch
    private DigitalInput intakeLimitSwitch;

    // Define Empty Arm PID Controllers
    // private final PIDController armRotateUpController;
    // private final PIDController armRotateDownController;

    public ArmSubsystem() {
        // Initialize Motor Objects to CAN SparkMAX ID
        intakeMotor = new CANSparkMax(14, MotorType.kBrushless);
        shooterTopMotor = new CANSparkMax(13, MotorType.kBrushless);
        shooterBottomMotor = new CANSparkMax(15, MotorType.kBrushless);

        intakeLimitSwitch = new DigitalInput(0);

        // Initalize ArmRotateUp PID Controller
        // double kPRotateUp = 0.0;
        // double kIRotateUp = 0.0; // Adjust accordingly
        // double kDRotateUp = 0.0;
        // armRotateUpController = new PIDController(kPRotateUp, kIRotateUp, kDRotateUp);
        // armRotateUpController.setSetpoint(0); // Adjust accordingly

        // // Initalize ArmRotateDown PID Controller
        // double kPRotateDown = 0.0;
        // double kIRotateDown = 0.0; // Adjust accordingly
        // double kDRotateDown = 0.0;
        // armRotateDownController = new PIDController(kPRotateDown, kIRotateDown, kDRotateDown);
        // armRotateDownController.setSetpoint(0); // Adjust accordingly
    }

    // Siccc LEDs
    public void lightsOn() {}
    public void lightsOff() {}

    public void ArmRotateUp() {}
    public void ArmRotateDown() {}
    public void ArmRotateStop() {}

    // Note Intake
    public void ArmIntake() {
        if (intakeLimitSwitch.get()) {
            intakeMotor.set(-0.5);
        } else {
            intakeMotor.set(0.0);
        }
    }
    
    // Note Outake
    public void ArmOuttake() {
        intakeMotor.set(1.0);
    }

    // Stopping Note Intake & Outake
    public void ArmIntakeStop() {
        intakeMotor.set(0.0);
    }

    // Start Shooter
    public void ArmShooter() {
        shooterTopMotor.set(0.7593);
        shooterBottomMotor.set(0.7593);
    }
    
    // Stop Shooter
    public void ArmShooterStop() {
        shooterTopMotor.set(0.0);
        shooterBottomMotor.set(0.0);
    }
}
