package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
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

    private Encoder armEncoderAngle;
    private int armGoalAngle;
    private int lastArmGoalAngle;

    private final PIDController armRotateUpController;
    private final PIDController armRotateDownController;
    private final PIDController armRotateTopController;
    private final PIDController armRotateBottomController;


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

        armEncoderAngle = new Encoder(-1, -1); // TODO: Adjust accordingly
        armEncoderAngle.reset();
        armGoalAngle = 0;

        // Initialize Arm Rotation PID Controllers
        double kPRotateUp = 0.0;
        double kIRotateUp = 0.0; // TODO: Adjust accordingly
        double kDRotateUp = 0.0;
        armRotateUpController = new PIDController(kPRotateUp, kIRotateUp, kDRotateUp);

        double kPRotateDown = 0.0;
        double kIRotateDown = 0.0; // TODO: Adjust accordingly
        double kDRotateDown = 0.0;
        armRotateDownController = new PIDController(kPRotateDown, kIRotateDown, kDRotateDown);

        double kPRotateTop = 0.0;
        double kIRotateTop = 0.0; // TODO: Adjust accordingly
        double kDRotateTop = 0.0;
        armRotateTopController = new PIDController(kPRotateTop, kIRotateTop, kDRotateTop);

        double kPRotateBottom = 0.0;
        double kIRotateBottom = 0.0; // TODO: Adjust accordingly
        double kDRotateBottom = 0.0;
        armRotateBottomController = new PIDController(kPRotateBottom, kIRotateBottom, kDRotateBottom);
    }

    // Set Arm Position for Ground Intaking
    public void ArmIntakePosition() {
        armGoalAngle = 0; // TODO: Test & Change these values
        lastArmGoalAngle = armGoalAngle;
        double armRotationSpeed = 0;

        if (armEncoderAngle.get() > 30) {
            armRotationSpeed = armRotateDownController.calculate(armEncoderAngle.get(), armGoalAngle);
        } else if (armEncoderAngle.get() > armGoalAngle) {
            armRotationSpeed = armRotateBottomController.calculate(armEncoderAngle.get(), armGoalAngle);
        } else {
            armEncoderAngle.reset();
        }
        ArmSetRotateSpeed(armRotationSpeed);
    }

    // Set Arm Position for Amp Scoring
    public void ArmAmpPosition() {
        armGoalAngle = 160; // TODO: Test & Change these values
        lastArmGoalAngle = armGoalAngle;
        double armRotationSpeed = 0;

        if (armEncoderAngle.get() < 130) {
            armRotationSpeed = armRotateUpController.calculate(armEncoderAngle.get(), armGoalAngle);
        } else if (armEncoderAngle.get() < armGoalAngle) {
            armRotationSpeed = armRotateTopController.calculate(armEncoderAngle.get(), armGoalAngle);
        } else {
            armRotationSpeed = armRotateDownController.calculate(armEncoderAngle.get(), armGoalAngle);
        }
        ArmSetRotateSpeed(armRotationSpeed);
    }

    // Stop Arm at Last Preset Position
    public void ArmRotateStop() {
        double armRotationSpeed = 0;

        if (armEncoderAngle.get() < lastArmGoalAngle) {
            armRotationSpeed = armRotateUpController.calculate(armEncoderAngle.get(), lastArmGoalAngle);
        } else if (armEncoderAngle.get() > lastArmGoalAngle) {
            armRotationSpeed = armRotateDownController.calculate(armEncoderAngle.get(), lastArmGoalAngle);
        }
        ArmSetRotateSpeed(armRotationSpeed);
    }

    public void ArmIntake() {
        if (intakeLimitSwitch.get()) {
            intakeMotor.set(-0.5);
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
        if (intakeLimitSwitch.get()) {
            shooterTopMotor.set(0.7593);
            shooterBottomMotor.set(0.7593);

            if (!shooterTimerStarted) {
                shooterTimer.start();
                shooterTimerStarted = true;
            }

            if (shooterTimer.get() >= 1.5) {
                intakeMotor.set(1.0);
            }
        }
    }
    
    public void ArmShooterStop() {
        shooterTopMotor.set(0.0);
        shooterBottomMotor.set(0.0);
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
