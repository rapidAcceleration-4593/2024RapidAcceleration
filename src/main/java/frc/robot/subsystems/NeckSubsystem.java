package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class NeckSubsystem extends SubsystemBase {

    // Neck Rotation
    private CANSparkMax leftGearbox1;
    private CANSparkMax leftGearbox2;
    private CANSparkMax rightGearbox1;
    private CANSparkMax rightGearbox2;

    private final Encoder neckEncoder;
    private int neckGoalAngle;
    private int lastNeckGoalAngle;

    private final PIDController neckRotateUpController;
    private final PIDController neckRotateDownController;
    private final PIDController neckRotateTopController;
    private final PIDController neckRotateBottomController;

    public NeckSubsystem() {
        // Initialize Motor Objects to CAN SparkMAX ID
        leftGearbox1 = new CANSparkMax(19, MotorType.kBrushless);
        leftGearbox2 = new CANSparkMax(20, MotorType.kBrushless);
        rightGearbox1 = new CANSparkMax(8, MotorType.kBrushless);
        rightGearbox2 = new CANSparkMax(9, MotorType.kBrushless);

        neckEncoder = new Encoder(8, 9);
        neckGoalAngle = 0;

        neckRotateUpController = new PIDController(0.001, 0.0025, 0.0001); // 0.01, 0.0025, 0.0001
        neckRotateDownController = new PIDController(0.00125, 0.0, 0.0);
        neckRotateTopController = new PIDController(0.00001, 0.0, 0.0);
        neckRotateBottomController = new PIDController(0.0013, 0.006, 0.0);
    }
    
    public void IntakePosition() {
        neckGoalAngle = 0;
        lastNeckGoalAngle = neckGoalAngle;
        double neckRotationSpeed = 0;

        if (neckEncoder.get() >= 20) {
            neckRotationSpeed = neckRotateDownController.calculate(neckEncoder.get(), neckGoalAngle);
        } else if (neckEncoder.get() > neckGoalAngle) {
            neckRotationSpeed = neckRotateBottomController.calculate(neckEncoder.get(), neckGoalAngle);
        } else {
            neckEncoder.reset();
        }
        NeckSetRotateSpeed(neckRotationSpeed);
    }

    public void SubwooferPosition() {
        neckGoalAngle = 85;
        lastNeckGoalAngle = neckGoalAngle;
        double neckRotationSpeed = 0;

        if (neckEncoder.get() < neckGoalAngle) {
            neckRotationSpeed = neckRotateUpController.calculate(neckEncoder.get(), neckGoalAngle);
        } else if (neckEncoder.get() > neckGoalAngle) {
            neckRotationSpeed = neckRotateDownController.calculate(neckEncoder.get(), neckGoalAngle);
        }
        NeckSetRotateSpeed(neckRotationSpeed);
    }

    public void AmpPosition() {
        neckGoalAngle = 200;
        lastNeckGoalAngle = neckGoalAngle;
        double neckRotationSpeed = 0;

        if (neckEncoder.get() <= 150) {
            neckRotationSpeed = neckRotateUpController.calculate(neckEncoder.get(), neckGoalAngle);
        } else if (neckEncoder.get() < neckGoalAngle) {
            neckRotationSpeed = neckRotateTopController.calculate(neckEncoder.get(), neckGoalAngle);
        } else {
            neckRotationSpeed = neckRotateDownController.calculate(neckEncoder.get(), neckGoalAngle);
        }
        NeckSetRotateSpeed(neckRotationSpeed);
    }


    public void NeckUp() {
        neckGoalAngle = neckEncoder.get();
        lastNeckGoalAngle = neckGoalAngle;

        NeckSetRotateSpeed(0.16);
    }

    public void NeckDown() {
        neckGoalAngle = neckEncoder.get();
        lastNeckGoalAngle = neckGoalAngle;
        
        NeckSetRotateSpeed(-0.06);
    }

    public void NeckHold() {
        double neckRotationSpeed = 0.0;

        if (neckEncoder.get() < lastNeckGoalAngle) {
            neckRotationSpeed = neckRotateUpController.calculate(neckEncoder.get(), lastNeckGoalAngle);
            System.out.println("Too low, go up");
        } else if (neckEncoder.get() > lastNeckGoalAngle) {
            neckRotationSpeed = neckRotateDownController.calculate(neckEncoder.get(), lastNeckGoalAngle);
            System.out.println("Too high, go down");
        }
        NeckSetRotateSpeed(neckRotationSpeed);
    }

    public void VisionNeckAngle() {
        boolean hasTargets = LimelightHelpers.getTV("");

        if (hasTargets && LimelightHelpers.getFiducialID("") == 4) {
            Pose3d target = LimelightHelpers.getTargetPose3d_CameraSpace("");
            
            System.out.println("Z Distance: " + target.getZ());
        }
    }

    private void NeckSetRotateSpeed(double speed) {
        leftGearbox1.set(speed);
        leftGearbox2.follow(leftGearbox1, false);
        rightGearbox1.follow(leftGearbox1, true);
        rightGearbox2.follow(leftGearbox1, true);

        System.out.println("<-------------------->");
        System.out.println("Set Value: " + lastNeckGoalAngle);
        System.out.println("Encoder Value: " + neckEncoder.get());
        System.out.println("Power: " + speed);
    }
}
