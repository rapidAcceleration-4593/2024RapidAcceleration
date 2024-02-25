package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.BeakConstants;
import frc.robot.Constants.NeckExtensionConstants;
import frc.robot.Constants.NeckRotationConstants;

public class NeckSubsystem extends SubsystemBase {

    private final CANSparkMax leftGearbox1;
    private final CANSparkMax leftGearbox2;
    private final CANSparkMax rightGearbox1;
    private final CANSparkMax rightGearbox2;
    private final CANSparkMax neckExtensionMotor;

    private final DigitalInput intakeLimitSwitch;
    // private final DigitalInput extensionTopLimitSwitch;
    private final DigitalInput extensionBottomLimitSwitch;
    private final DigitalInput bottomLimitSwitch;

    private final Encoder neckEncoder;
    private final DutyCycleEncoder throughBoreEncoder;

    public int neckGoalAngle = 0;
    public int lastNeckGoalAngle = neckGoalAngle;

    private boolean intakeDrivingHold = false;

    public final PIDController neckRotateController = new PIDController(0.0, 0.0, 0.0);;

    public NeckSubsystem() {
        // Initialize Motor Objects to CAN SparkMAX ID
        leftGearbox1 = NeckRotationConstants.leftGearbox1;
        leftGearbox2 = NeckRotationConstants.leftGearbox2;
        rightGearbox1 = NeckRotationConstants.rightGearbox1;
        rightGearbox2 = NeckRotationConstants.rightGearbox2;
        neckExtensionMotor = NeckExtensionConstants.neckExtensionMotor;

        intakeLimitSwitch = BeakConstants.intakeLimitSwitch;
        // extensionTopLimitSwitch = NeckExtensionConstants.extensionTopLimitSwitch;
        extensionBottomLimitSwitch = NeckExtensionConstants.extensionBottomLimitSwitch;
        bottomLimitSwitch = NeckRotationConstants.bottomLimitSwitch;

        neckEncoder = NeckRotationConstants.neckEncoder;
        throughBoreEncoder = NeckExtensionConstants.throughBoreEncoder;
    }

    public void NeckUp() {
        // NeckSetRotateSpeed(0.16);
        
        // neckGoalAngle = neckEncoder.get();
        // lastNeckGoalAngle = neckGoalAngle;
        System.out.println("Bottom Arm Switch: " + bottomLimitSwitch.get());
        System.out.println("Bottom Extension Switch: " + extensionBottomLimitSwitch.get());
    }

    public void NeckDown() {
        NeckSetRotateSpeed(-0.06);

        neckGoalAngle = neckEncoder.get();
        lastNeckGoalAngle = neckGoalAngle;
    }

    public void IntakePosition() {
        neckGoalAngle = 0;
        lastNeckGoalAngle = neckGoalAngle;
    }

    public void DrivingPosition() {
        neckGoalAngle = 40;
        lastNeckGoalAngle = neckGoalAngle;
    }

    public void SubwooferPosition() {
        neckGoalAngle = 80; // 50
        lastNeckGoalAngle = neckGoalAngle;
    }

    public void AmpPosition() {
        neckGoalAngle = 210;
        lastNeckGoalAngle = neckGoalAngle;
    }

    public void NeckFollower() {
        if (!bottomLimitSwitch.get()) {
            neckEncoder.reset();
        }

        // updateExtensionAngle();
        updatePIDConstants();

        // if (neckGoalAngle < 30 && neckEncoder.get() >= 30 && throughBoreEncoder.get() > -0.75) {
        //     // dont go down yet
        // } else if (neckGoalAngle < 30 && neckEncoder.get() >= 30 && throughBoreEncoder.get() < -0.75) {
        //     // now you can go down
        //     updatePIDConstants();
        // } else if (neckGoalAngle > 30 && neckEncoder.get() <= 30 && throughBoreEncoder.get() < -0.75) {
        //     // dont go up yet
        // } else if (neckGoalAngle > 30 && neckEncoder.get() <= 30 && throughBoreEncoder.get() > -0.75) {
        //     updatePIDConstants();
        // } else {
        //     updatePIDConstants();
        // }

        
        double neckRotationSpeed = neckRotateController.calculate(neckEncoder.get(), lastNeckGoalAngle);
        NeckSetRotateSpeed(neckRotationSpeed);
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

    private void updateExtensionAngle() {
        // if (neckGoalAngle == 0 || neckGoalAngle == 210) {
        //     ExtendOut();
        // } else if (neckEncoder.get() > 30 && neckEncoder.get() < 220) {
        //     ExtendIn();
        // } else {
        //     ExtendStop();
        // }
    }

    private void updatePIDConstants() {
        double p, i, d;

        // Calculate PID Constants based on the neck encoder value
        if (neckEncoder.get() <= 30 && neckGoalAngle <= 30) {
            // Bottom Controller
            p = 0.0013;
            i = 0.0;
            d = 0.0;
        } else if (neckEncoder.get() >= 195 && neckGoalAngle >= 195) {
            // Top Controller
            p = 0.00005;
            i = 0.00005;
            d = 0.0;
        } else if (neckEncoder.get() > neckGoalAngle - 3 && neckEncoder.get() < neckGoalAngle + 2) {
            // Close Controller
            p = 0.00005;
            i = 0.0005;
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

    private int visionSetAngle(double distance) {
        int angle = 0;
        if (distance > 3.5 && distance < 8) {
            angle = (int) (0.666667*Math.pow(distance, 3) + -15.0303*Math.pow(distance, 2) + 109.106 * distance + -145.079);
        }
        return angle;
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


    public void ExtendIn() {
        // if (!extensionBottomLimitSwitch.get()) {
        //     neckExtensionMotor.set(0.0);
        //     throughBoreEncoder.reset();
        // } else {
        //     neckExtensionMotor.set(0.5);
        // }
        // System.out.println("Absolute Position: " + throughBoreEncoder.get());
    }

    public void ExtendOut() {
        // if (throughBoreEncoder.get() <= -1.25) {
        //     neckExtensionMotor.set(0.0);
        // } else {
        //     neckExtensionMotor.set(-0.5);
        // }
        // System.out.println("Absolute Position: " + throughBoreEncoder.get());
    }

    public void ExtendStop() {
        // neckExtensionMotor.set(0.0);
    }

    public void periodic() {
        if (!extensionBottomLimitSwitch.get()) {
            throughBoreEncoder.reset();
        }





        // if (!intakeLimitSwitch.get()) {
        //     DrivingPosition();
        //     NeckFollower();
        // }
    }
}
