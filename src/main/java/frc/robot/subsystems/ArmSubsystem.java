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

    private final Encoder armEncoder;
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

        armEncoder = new Encoder(8, 9);
        armGoalAngle = 0;

        armRotateUpController = new PIDController(0.01, 0.0022, 0.0);
        armRotateDownController = new PIDController(0.001, 0.0, 0.0);
        armRotateTopController = new PIDController(0.0, 0.0, 0.0);
        armRotateBottomController = new PIDController(0.0, 0.0, 0.0);
    }
    
    public void IntakePosition() {
        // armGoalAngle = 0;
        // lastArmGoalAngle = armGoalAngle;
        // double armRotationSpeed = 0;

        // if (armEncoder.get() >= 30) {
        //     armRotationSpeed = armRotateDownController.calculate(armEncoder.get(), armGoalAngle);
        // } else if (armEncoder.get() > armGoalAngle) {
        //     armRotationSpeed = armRotateBottomController.calculate(armEncoder.get(), armGoalAngle);
        // } else {
        //     armEncoder.reset();
        // }
        // ArmSetRotateSpeed(armRotationSpeed);
    }

    public void SubwooferPosition() {
        // armGoalAngle = 85;
        // lastArmGoalAngle = armGoalAngle;
        // double armRotationSpeed = 0;

        // if (armEncoder.get() < armGoalAngle) {
        //     armRotationSpeed = armRotateUpController.calculate(armEncoder.get(), armGoalAngle);
        // } else if (armEncoder.get() > armGoalAngle) {
        //     armRotationSpeed = armRotateDownController.calculate(armEncoder.get(), armGoalAngle);
        // }
        // ArmSetRotateSpeed(armRotationSpeed);
    }

    public void AmpPosition() {
        // armGoalAngle = 300;
        // lastArmGoalAngle = armGoalAngle;
        // double armRotationSpeed = 0;

        // if (armEncoder.get() <= 270) {
        //     armRotationSpeed = armRotateUpController.calculate(armEncoder.get(), armGoalAngle);
        // } else if (armEncoder.get() < armGoalAngle) {
        //     armRotationSpeed = armRotateTopController.calculate(armEncoder.get(), armGoalAngle);
        // } else {
        //     armRotationSpeed = armRotateDownController.calculate(armEncoder.get(), armGoalAngle);
        // }
        // ArmSetRotateSpeed(armRotationSpeed);
    }


    public void ArmUp() {
        armGoalAngle = armEncoder.get();
        lastArmGoalAngle = armGoalAngle;

        ArmSetRotateSpeed(0.16);
    }

    public void ArmDown() {
        armGoalAngle = armEncoder.get();
        lastArmGoalAngle = armGoalAngle;
        
        ArmSetRotateSpeed(-0.06);
    }

    public void ArmRotateStop() {
        double armRotationSpeed = 0.0;

        if (armEncoder.get() < lastArmGoalAngle) {
            // armRotationSpeed = armRotateUpController.calculate(armEncoder.get(), lastArmGoalAngle);
            System.out.println("Too low, go up");
        } else if (armEncoder.get() > lastArmGoalAngle) {
            // armRotationSpeed = armRotateDownController.calculate(armEncoder.get(), lastArmGoalAngle);
            System.out.println("Too high, go down");
        }
        ArmSetRotateSpeed(armRotationSpeed);
    }



    // Intake, Outtake, and Shooter
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
        if (armEncoder.get() < 300) {
            shooterTopMotor.set(0.7593);
            shooterBottomMotor.set(0.7593);
        } else {
            shooterTopMotor.set(0.25);
            shooterBottomMotor.set(0.25);
        }

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

        System.out.println("<-------------------->");
        System.out.println("Set Value: " + lastArmGoalAngle);
        System.out.println("Encoder Value: " + armEncoder.get());
        System.out.println("Power: " + speed);
    }
}
