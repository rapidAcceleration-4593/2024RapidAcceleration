package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Encoder;
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

    private final PIDController neckRotateController;

    public NeckRotationSubsystem() {
        // Initialize Motor Objects to CAN SparkMAX ID
        leftGearbox1 = new CANSparkMax(19, MotorType.kBrushless);
        leftGearbox2 = new CANSparkMax(20, MotorType.kBrushless);
        rightGearbox1 = new CANSparkMax(8, MotorType.kBrushless);
        rightGearbox2 = new CANSparkMax(9, MotorType.kBrushless);

        neckEncoder = new Encoder(8, 9);
        neckGoalAngle = 0;

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
        neckGoalAngle = 85;
        lastNeckGoalAngle = neckGoalAngle;
    }

    public void AmpPosition() {
        neckGoalAngle = 200;
        lastNeckGoalAngle = neckGoalAngle;
    }

    public void NeckFollower() {
        updatePIDConstants();
        applyPIDController(lastNeckGoalAngle);
    }

    public void VisionNeckAngle() {
        boolean hasTargets = LimelightHelpers.getTV("");

        if (hasTargets && LimelightHelpers.getFiducialID("") == 4) {
            Pose3d target = LimelightHelpers.getTargetPose3d_CameraSpace("");
            
            System.out.println("Z Distance: " + target.getZ());

            updatePIDConstants();
            applyPIDController(visionSetAngle(target.getZ()));
        }
    }

    private int visionSetAngle(double distance) {
        int angle = 0;
        // if (distance > 0 && distance < 5) {
        //     angle = 0; // Enter Equation with variable distance as unknown variable x - cast as int
        // }
        return angle;
    }

    private void updatePIDConstants() {
        double p, i, d;

        // Calculate PID Constants based on the neck encoder value
        if (neckEncoder.get() <= 20) {
            // Bottom Controller
            p = 0.0013;
            i = 0.006;
            d = 0.0;
        } else if (neckEncoder.get() >= 160) {
            // Top Controller
            p = 0.00001;
            i = 0.0;
            d = 0.0;
        } else if (neckEncoder.get() < neckGoalAngle) {
            // Up Controller
            p = 0.01;
            i = 0.0025;
            d = 0.0001;
        } else if (neckEncoder.get() > neckGoalAngle) {
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
