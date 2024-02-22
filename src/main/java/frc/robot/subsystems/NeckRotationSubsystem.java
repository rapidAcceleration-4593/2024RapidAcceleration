package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class NeckRotationSubsystem extends SubsystemBase {

    private final CANSparkMax leftGearbox1;
    private final CANSparkMax leftGearbox2;
    private final CANSparkMax rightGearbox1;
    private final CANSparkMax rightGearbox2;

    private final Encoder neckEncoder;
    private int neckGoalAngle;
    private int lastNeckGoalAngle;

    private DigitalInput bottomLimitSwitch;

    private final PIDController neckRotateController;

    public NeckRotationSubsystem() {
        // Initialize Motor Objects to CAN SparkMAX ID
        leftGearbox1 = new CANSparkMax(19, MotorType.kBrushless);
        leftGearbox2 = new CANSparkMax(20, MotorType.kBrushless);
        rightGearbox1 = new CANSparkMax(8, MotorType.kBrushless);
        rightGearbox2 = new CANSparkMax(9, MotorType.kBrushless);

        neckEncoder = new Encoder(8, 9);
        neckGoalAngle = 0;

        bottomLimitSwitch = new DigitalInput(1);

        neckRotateController = new PIDController(0.0, 0.0, 0.0);
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

    public void IntakePosition() {
        neckGoalAngle = 0;
        lastNeckGoalAngle = neckGoalAngle;
    }

    public void SubwooferPosition() {
        neckGoalAngle = 80;
        lastNeckGoalAngle = neckGoalAngle;
    }

    public void AmpPosition() {
        neckGoalAngle = 200;
        lastNeckGoalAngle = neckGoalAngle;
    }

    public void NeckFollower() {
        updatePIDConstants();
        applyPIDController(lastNeckGoalAngle);

        if (!bottomLimitSwitch.get()) {
            neckEncoder.reset();
        }
    }

    public void VisionNeckAngle() {
        boolean hasTargets = LimelightHelpers.getTV("");

        if (hasTargets && LimelightHelpers.getFiducialID("") == 4) {
            Pose3d target = LimelightHelpers.getTargetPose3d_CameraSpace("");
            
            System.out.println("Z Distance: " + target.getZ());

            neckGoalAngle = visionSetAngle(target.getZ());
            lastNeckGoalAngle = neckGoalAngle;
        }
    }

    private int visionSetAngle(double distance) {
        int angle = 0;
        if (distance > 3.5 && distance < 8) {
            angle = (int) (0.666667*Math.pow(distance, 3) + -15.0303*Math.pow(distance, 2) + 109.106 * distance + -145.079);
        }
        return angle;
    }

    private void updatePIDConstants() {
        double p, i, d;

        // Calculate PID Constants based on the neck encoder value
        if (neckEncoder.get() <= 30 && neckGoalAngle <= 30) {
            // Bottom Controller
            p = 0.0013;
            i = 0.0;
            d = 0.0;
        } else if (neckEncoder.get() >= 180 && neckGoalAngle >= 180) {
            // Top Controller
            p = 0.00005;
            i = 0.00001;
            d = 0.0;
        } else if (neckEncoder.get() > neckGoalAngle - 2 && neckEncoder.get() < neckGoalAngle + 2) {
            // Close Controller
            p = 0.00005;
            i = 0.001;
            d = 0.0;
        } else if (neckEncoder.get() < neckGoalAngle) {
            // Up Controller
            p = 0.002;
            i = 0.0075;
            d = 0.0002;
        } else if (neckEncoder.get() > neckGoalAngle && neckEncoder.get() > 0) {
            // Down Controller
            p = 0.00125;
            i = 0.0;
            d = 0.0;
        } else {
            // Default Controller
            p = 0.0;
            i = 0.0;
            d = 0.0;
        }

        neckRotateController.setPID(p, i, d);
    }

    private void applyPIDController(double goal) {
        double neckRotationSpeed = neckRotateController.calculate(neckEncoder.get(), goal);
        NeckSetRotateSpeed(neckRotationSpeed);
    }

    private void NeckSetRotateSpeed(double speed) {
        leftGearbox1.set(speed);
        leftGearbox2.follow(leftGearbox1, false);
        rightGearbox1.follow(leftGearbox1, true);
        rightGearbox2.follow(leftGearbox1, true);

        System.out.println("<-------------------->");
        System.out.println("Encoder Value: " + neckEncoder.get());
        System.out.println("Set Value: " + lastNeckGoalAngle);
        System.out.println("Power: " + speed);
    }
}
