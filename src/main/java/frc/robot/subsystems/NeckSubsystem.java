package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.*;

public class NeckSubsystem extends SubsystemBase {

    private final CANSparkMax leftGearbox1;
    private final CANSparkMax leftGearbox2;
    private final CANSparkMax rightGearbox1;
    private final CANSparkMax rightGearbox2;
    private final CANSparkMax neckExtensionMotor;

    private final DigitalInput intakeLimitSwitch;
    private final DigitalInput extensionTopLimitSwitch;
    private final DigitalInput extensionBottomLimitSwitch;
    private final DigitalInput topLimitSwitch;
    private final DigitalInput bottomLimitSwitch;

    public final PIDController neckRotateController = new PIDController(0.0, 0.0, 0.0);
    public final PIDController neckRotateDownInit = new PIDController(0.0, 0.0, 0.0);

    private final Encoder neckEncoder;
    private final DutyCycleEncoder throughBoreEncoder;

    private boolean neckInitialized = false;
    private boolean drivingPositionSet = false;
    private boolean initializeTimerStarted = false;
    private boolean bottomLimitSwitchState = false;
    private boolean runNeckInitialization;

    private int extensionState = 1;
    private int speakerAprilTagID;

    private int neckGoalAngle = 0;
    private int neckAutoGoalAngle = 0;
    private int lastNeckGoalAngle = neckGoalAngle;

    private Timer initializeTimer = new Timer();

    public NeckSubsystem() {
        leftGearbox1 = NeckRotationConstants.leftGearbox1;
        leftGearbox2 = NeckRotationConstants.leftGearbox2;
        rightGearbox1 = NeckRotationConstants.rightGearbox1;
        rightGearbox2 = NeckRotationConstants.rightGearbox2;
        neckExtensionMotor = NeckExtensionConstants.neckExtensionMotor;

        intakeLimitSwitch = BeakConstants.intakeLimitSwitch;
        extensionTopLimitSwitch = NeckExtensionConstants.extensionTopLimitSwitch;
        extensionBottomLimitSwitch = NeckExtensionConstants.extensionBottomLimitSwitch;
        topLimitSwitch = NeckRotationConstants.topLimitSwitch;
        bottomLimitSwitch = NeckRotationConstants.bottomLimitSwitch;

        neckEncoder = NeckRotationConstants.neckEncoder;
        throughBoreEncoder = NeckExtensionConstants.throughBoreEncoder;

        runNeckInitialization = MatchConstants.runNeckInitialization;

        speakerAprilTagID = MatchConstants.speakerAprilTagID;
    }

    public void NeckDownInit() {
        if (!neckInitialized && runNeckInitialization && !extensionBottomLimitSwitch.get() && bottomLimitSwitch.get() && topLimitSwitch.get()) {
            evaluateExtension();

            if (!initializeTimerStarted && neckEncoder.get() <= -170 && neckEncoder.get() >= -175) {
                initializeTimer.start();
            }

            if (initializeTimer.get() >= 1.25) {
                if (extensionState != 1) {
                    ExtendOut();
                    neckAutoGoalAngle = -175;
                } else {
                    ExtendStop();
                    neckAutoGoalAngle = -220;
                }
            } else {
                neckAutoGoalAngle = -175;
            }

            updateAutoPIDConstants();

            double neckRotationSpeed = neckRotateDownInit.calculate(neckEncoder.get(), neckAutoGoalAngle);
            NeckSetRotateSpeed(neckRotationSpeed);
        } else {
            System.out.println("Robot Not Initialized Properly");
            
            // Extend Out (if needed), fixed speed to bottom limit switch
        }
    }

    public void IntakePosition() {
        if (neckInitialized && intakeLimitSwitch.get() && lastNeckGoalAngle != 270) {
            neckGoalAngle = 0;
            lastNeckGoalAngle = neckGoalAngle;
        }
    }

    public void SubwooferPosition() {
        if (neckInitialized) {
            neckGoalAngle = 55;
            lastNeckGoalAngle = neckGoalAngle;
        }
    }

    public void AmpPosition() {
        if (neckInitialized) {
            neckGoalAngle = 270;
            lastNeckGoalAngle = neckGoalAngle;
        }
    }

    private void NeckFollower() {
        int goal = lastNeckGoalAngle;

        if (neckInitialized) {
            evaluateExtension();

            if (neckGoalAngle < 30) {
                if (extensionState != 1) {
                    ExtendOut();
                    lastNeckGoalAngle = 40;
                } else {
                    ExtendStop();
                }
            } else if (neckGoalAngle > 180) {
                if (extensionState != 1) {
                    ExtendOut();
                } else {
                    ExtendStop();
                }
            } else {
                if (extensionState != 2 && neckEncoder.get() > 30) {
                    ExtendIn();
                } else {
                    ExtendStop();
                }
            }

            updatePIDConstants();

            double neckRotationSpeed = neckRotateController.calculate(neckEncoder.get(), lastNeckGoalAngle);
            NeckSetRotateSpeed(neckRotationSpeed);

            lastNeckGoalAngle = goal;
        }
    }

    private void evaluateExtension() {
        if (!extensionTopLimitSwitch.get()) {
            // Extended Out
            extensionState = 1;
        } else if (!extensionBottomLimitSwitch.get()) {
            // Extended In
            extensionState = 2;
        } else {
            // In motion
            extensionState = 0;
        }
    }

    private void updatePIDConstants() {
        double p, i, d;

        // Calculate PID Constants based on the neck encoder value
        if (neckEncoder.get() <= 30 && neckEncoder.get() > 12 && neckGoalAngle <= 30) {
            // Bottom Controller
            p = 0.00375;
            i = 0.00225;
            d = 0.0;
        } else if (neckEncoder.get() > neckGoalAngle - 4 && neckEncoder.get() < neckGoalAngle + 3) {
            // Close Controller
            p = 0.0001;
            i = 0.0013;
            d = 0.0;
        } else if (neckEncoder.get() < neckGoalAngle && neckEncoder.get() < 195) {
            // Up Controller
            p = 0.002;
            i = 0.0075;
            d = 0.0002;
        } else if (neckEncoder.get() > neckGoalAngle && neckEncoder.get() > 30) {
            // Down Controller
            p = 0.0016;
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

    private void updateAutoPIDConstants() {
        double p, i, d;

        if (neckEncoder.get() <= 0 && neckEncoder.get() > neckAutoGoalAngle - 3 && neckEncoder.get() < neckAutoGoalAngle + 2) {
            // Close Controller
            p = 0.000075;
            i = 0.00075;
            d = 0.0;
        } else if (neckEncoder.get() <= 0 && neckEncoder.get() >= -180 && neckAutoGoalAngle == -175) {
            // Down to Subwoofer
            p = 0.0016;
            i = 0.0;
            d = 0.0;
        } else if (neckEncoder.get() <= 0 && neckEncoder.get() < -170 && neckAutoGoalAngle == -220) {
            // Down to Intake
            p = 0.0015;
            i = 0.0001;
            d = 0.0;
        } else {
            p = 0.0;
            i = 0.0;
            d = 0.0;
        }

        neckRotateDownInit.setPID(p, i, d);
    }

    public void VisionNeckAngle() {
        boolean hasTargets = LimelightHelpers.getTV("");

        if (hasTargets && LimelightHelpers.getFiducialID("") == speakerAprilTagID) {
            Pose3d target = LimelightHelpers.getTargetPose3d_CameraSpace("");
            
            System.out.println("Z Distance: " + target.getZ());

            neckGoalAngle = visionSetAngle(target.getZ());
            lastNeckGoalAngle = neckGoalAngle;
        }
    }

    private int visionSetAngle(double distance) {
        int angle = 0;
        if (distance > 3.3 && distance < 10) {
            angle = (int) (-349.351*Math.pow(distance, -0.854219) + 180.059);
        }
        return angle;
    }

    private void NeckSetRotateSpeed(double speed) {
        leftGearbox1.set(speed);
        leftGearbox2.follow(leftGearbox1, false);
        rightGearbox1.follow(leftGearbox1, true);
        rightGearbox2.follow(leftGearbox1, true);
    }

    public void ExtendIn() {
        neckExtensionMotor.set(0.5);
    }

    public void ExtendOut() {
        neckExtensionMotor.set(-0.5);
    }

    public void ExtendStop() {
        neckExtensionMotor.set(0.0);
    }

    public void NeckUp() {
        NeckSetRotateSpeed(0.16);
        neckGoalAngle = neckEncoder.get();
        lastNeckGoalAngle = neckGoalAngle;
    }

    public void NeckDown() {
        NeckSetRotateSpeed(-0.05);
        neckGoalAngle = neckEncoder.get();
        lastNeckGoalAngle = neckGoalAngle;
    }

    public void NeckStop() {
        NeckSetRotateSpeed(0.0);
    }

    public void periodic() {
        NeckDownInit();
        NeckFollower();

        if (topLimitSwitch.get()) {
            if (neckEncoder.get() >= 195 && neckEncoder.get() < 275) {
                NeckSetRotateSpeed(0.01);
            }
        }

        if (!bottomLimitSwitch.get()) {
            if (!bottomLimitSwitchState) {
                neckEncoder.reset();
                bottomLimitSwitchState = true;
            }
            neckInitialized = true;
        } else {
            bottomLimitSwitchState = false;
            if ((neckEncoder.get() <= 12 && neckEncoder.get() > 0 && lastNeckGoalAngle == 0) || (neckEncoder.get() <= -210 && lastNeckGoalAngle < 200)) {
                NeckSetRotateSpeed(-0.05);
            }
        }

        if (!intakeLimitSwitch.get() && neckEncoder.get() <= 30) { //  && initializeTimer.get() >= 15
            if (!drivingPositionSet) {
                SubwooferPosition();
                drivingPositionSet = true;
            }
        } else {
            drivingPositionSet = false;
        }

        if (!extensionBottomLimitSwitch.get()) {
            throughBoreEncoder.reset();
        }

        System.out.println("<--------------->");
        System.out.println("Goal:" + lastNeckGoalAngle);
        System.out.println("Encoder Value: " + neckEncoder.get());
    }
}
