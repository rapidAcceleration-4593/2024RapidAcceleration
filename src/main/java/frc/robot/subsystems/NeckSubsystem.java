package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.*;

public class NeckSubsystem extends SubsystemBase {

    private final CANSparkMax leftGearbox1 = NeckRotationConstants.leftGearbox1;
    private final CANSparkMax leftGearbox2 = NeckRotationConstants.leftGearbox2;
    private final CANSparkMax rightGearbox1 = NeckRotationConstants.rightGearbox1;
    private final CANSparkMax rightGearbox2 = NeckRotationConstants.rightGearbox2;
    private final CANSparkMax neckExtensionMotor = NeckExtensionConstants.neckExtensionMotor;

    private final DigitalInput intakeLimitSwitch = BeakConstants.intakeLimitSwitch;
    private final DigitalInput extensionTopLimitSwitch = NeckExtensionConstants.extensionTopLimitSwitch;
    private final DigitalInput extensionBottomLimitSwitch = NeckExtensionConstants.extensionBottomLimitSwitch;
    private final DigitalInput topLimitSwitch = NeckRotationConstants.topLimitSwitch;
    private final DigitalInput bottomLimitSwitch = NeckRotationConstants.bottomLimitSwitch;

    private final Encoder neckEncoder = NeckRotationConstants.neckEncoder;
    private final Encoder backupNeckEncoder = NeckRotationConstants.backupNeckEncoder;
    private final DutyCycleEncoder throughBoreEncoder = NeckExtensionConstants.throughBoreEncoder;

    private boolean runNeckInitialization = MatchConstants.runNeckInitialization;;
    private boolean neckInitialized = false;
    private boolean drivingPositionSet = false;
    private boolean initializeTimerStarted = false;
    private boolean bottomLimitSwitchState = false;
    private boolean manualControlEnabled = false;

    public final PIDController neckRotateController = new PIDController(0.0, 0.0, 0.0);
    public final PIDController neckRotateDownInit = new PIDController(0.0, 0.0, 0.0);

    private int extensionState = 1;
    private int neckGoalAngle = 0;
    private int neckAutoGoalAngle = 0;
    private int lastNeckGoalAngle = neckGoalAngle;

    private Timer initializeTimer = new Timer();

    private enum NeckStates {
        INTAKE,
        DRIVE,
        SUBWOOFER,
        VISION,
        YEET,
        AMP,
        MANUAL_UP,
        MANUAL_DOWN,
        MANUAL_STOP
    }

    private NeckStates currentNeckState;

    private void NeckInitialization() {
        if (!neckInitialized && runNeckInitialization && bottomLimitSwitch.get()) {
            if (extensionState == 2) {
                if (!initializeTimerStarted && neckEncoder.get() <= -150) {
                    initializeTimer.start();
                }

                if (initializeTimer.get() >= 0.8) {
                    if (extensionState != 1) {
                        ExtendOut();
                        neckAutoGoalAngle = -175;
                    }
                } else {
                    neckAutoGoalAngle = -175;
                }
            } else if (extensionState == 1) {
                ExtendStop();
                neckAutoGoalAngle = -220;
            }

            UpdateAutoPIDConstants();

            double neckRotationSpeed = neckRotateDownInit.calculate(neckEncoder.get(), neckAutoGoalAngle);
            NeckSetRotateSpeed(neckRotationSpeed);
        }
    }

    public void IntakePosition() { currentNeckState = NeckStates.INTAKE; }
    public void SubwooferPosition() { currentNeckState = NeckStates.SUBWOOFER; }
    public void VisionNeckAngle() { currentNeckState = NeckStates.VISION; }
    public void YeetPosition() { currentNeckState = NeckStates.YEET; }
    public void AmpPosition() { currentNeckState = NeckStates.AMP; }

    public void NeckUp() { currentNeckState = NeckStates.MANUAL_UP; }
    public void NeckDown() { currentNeckState = NeckStates.MANUAL_DOWN; }
    public void NeckStop() { currentNeckState = NeckStates.MANUAL_STOP; }

    public void EnableManualControl() { manualControlEnabled = true; }
    public void DisableManualControl() { manualControlEnabled = false; }

    public void ExtendIn() { neckExtensionMotor.set(0.5); }
    public void ExtendOut() { neckExtensionMotor.set(-0.5); }
    public void ExtendStop() { neckExtensionMotor.set(0.0); }

    private void NeckFollower() {
        int goal = lastNeckGoalAngle;

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

        UpdatePIDConstants();

        double neckRotationSpeed = neckRotateController.calculate(neckEncoder.get(), lastNeckGoalAngle);
        NeckSetRotateSpeed(neckRotationSpeed);

        lastNeckGoalAngle = goal;
    }

    private void EvaluateExtension() {
        if (!extensionTopLimitSwitch.get()) { // throughBoreEncoder.get() <= -1
            // Extended Out
            extensionState = 1;
        } else if (!extensionBottomLimitSwitch.get()) { // || throughBoreEncoder.get() >= -0.2
            // Extended In
            extensionState = 2;
        } else {
            // In motion
            extensionState = 0;
        }
    }

    private void UpdatePIDConstants() {
        double p, i, d;

        if (neckEncoder.get() > neckGoalAngle - 4 && neckEncoder.get() < neckGoalAngle + 3 && neckGoalAngle > 5 && neckEncoder.get() > 5) {
            // Close Controller
            p = 0.00005;
            i = 0.0013;
            d = 0.0008;
        } else if (neckEncoder.get() < neckGoalAngle && neckEncoder.get() < 235 && neckGoalAngle > 5) {
            // Up Controller
            p = 0.0019;
            i = 0.0075;
            d = 0.0002;
        } else if (neckEncoder.get() > neckGoalAngle && neckEncoder.get() > 25 && neckGoalAngle > 5) {
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

    private void UpdateAutoPIDConstants() {
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

    private int VisionSetAngle(double distance) {
        int angle = 0;
        if (distance > 3.3 && distance < 13) {
            angle = (int) (-349.351*Math.pow(distance, -0.854219) + 200.0);
        }
        return angle;
    }

    private void NeckSetRotateSpeed(double speed) {
        leftGearbox1.set(speed);
        leftGearbox2.follow(leftGearbox1, false);
        rightGearbox1.follow(leftGearbox1, true);
        rightGearbox2.follow(leftGearbox1, true);
    }

    public void periodic() {
        EvaluateExtension();
        // ManageEncoderFailure();

        if (!neckInitialized && runNeckInitialization && bottomLimitSwitch.get()) {
            NeckInitialization();
        } else if (neckInitialized || !bottomLimitSwitch.get() || manualControlEnabled) {
            neckInitialized = true;
            ManageNeckStates();
        }

        if (extensionState == 2) {
            throughBoreEncoder.reset();
        }

        if (!manualControlEnabled) {
            if (neckInitialized) {
                NeckFollower();

                if (topLimitSwitch.get()) {
                    if (neckEncoder.get() >= 235 && neckEncoder.get() < 280) {
                        NeckSetRotateSpeed(0.015);
                    }
                }

                if (!intakeLimitSwitch.get() && neckEncoder.get() <= 30) {
                    if (!drivingPositionSet) {
                        currentNeckState = NeckStates.DRIVE;
                        drivingPositionSet = true;
                    }
                } else {
                    drivingPositionSet = false;
                }
            }

            if (!bottomLimitSwitch.get()) {
                if (!bottomLimitSwitchState) {
                    neckEncoder.reset();
                    bottomLimitSwitchState = true;
                }
            } else {
                bottomLimitSwitchState = false;
                if ((neckEncoder.get() <= 25 && neckEncoder.get() > 0 && lastNeckGoalAngle == 0) || (neckEncoder.get() <= -210 && lastNeckGoalAngle < 200)) {
                    NeckSetRotateSpeed(-0.05);
                }
            }
        } else {
            if (extensionState != 1) {
                ExtendOut();
            } else {
                ExtendStop();
            }
        }

        System.out.println("<--------------->");
        System.out.println("Goal:" + lastNeckGoalAngle);
        System.out.println("Encoder Value: " + neckEncoder.get());
        System.out.println("Backup Encoder Value: " + backupNeckEncoder.get());

        System.out.println("Top Limit Switch: " + topLimitSwitch.get());
        System.out.println("Bottom Limit Switch: " + bottomLimitSwitch.get());
        System.out.println("Extension Top Limit Switch: " + extensionTopLimitSwitch.get());
        System.out.println("Extension Bottom Limit Switch: " + extensionBottomLimitSwitch.get());
        System.out.println("Intake Limit Switch: " + intakeLimitSwitch.get());

        // System.out.println("Through Bore Encoder: " + throughBoreEncoder.get());

        System.out.println("Manual Control Enabled: " + manualControlEnabled);
    }

    private void ManageNeckStates() {
        switch (currentNeckState) {
            case INTAKE:
                if (!manualControlEnabled && intakeLimitSwitch.get() && lastNeckGoalAngle != 270) {
                    neckGoalAngle = 0;
                    lastNeckGoalAngle = neckGoalAngle;
                }
                break;
            case DRIVE:
                if (!manualControlEnabled) {
                    neckGoalAngle = 55;
                    lastNeckGoalAngle = neckGoalAngle;
                }
                break;
            case SUBWOOFER:
                if (!manualControlEnabled) {
                    neckGoalAngle = 80;
                    lastNeckGoalAngle = neckGoalAngle;
                }
                break;
            case VISION:
                if (!manualControlEnabled) {
                    boolean hasTargets = LimelightHelpers.getTV("");
                    if (hasTargets && (LimelightHelpers.getFiducialID("") == 4 || LimelightHelpers.getFiducialID("") == 7)) {
                        neckGoalAngle = VisionSetAngle(LimelightHelpers.getTargetPose3d_CameraSpace("").getZ());
                        lastNeckGoalAngle = neckGoalAngle;
                    }
                }
                break;
            case YEET:
                if (!manualControlEnabled) {
                    neckGoalAngle = 120;
                    lastNeckGoalAngle = neckGoalAngle;
                }
                break;
            case AMP:
                if (!manualControlEnabled && lastNeckGoalAngle != 0) {
                    neckGoalAngle = 270;
                    lastNeckGoalAngle = neckGoalAngle;
                }
                break;
            case MANUAL_UP:
                if (manualControlEnabled) {
                    if (topLimitSwitch.get()) {
                        NeckSetRotateSpeed(0.16);
                        neckGoalAngle = neckEncoder.get();
                        lastNeckGoalAngle = neckGoalAngle;
                    } else {
                        NeckStop();
                    }
                }
                break;
            case MANUAL_DOWN:
                if (manualControlEnabled) {
                    if (bottomLimitSwitch.get()) {
                        NeckSetRotateSpeed(-0.06);
                        neckGoalAngle = neckEncoder.get();
                        lastNeckGoalAngle = neckGoalAngle;
                    } else {
                        NeckStop();
                    }
                }
                break;
            case MANUAL_STOP:
                if (manualControlEnabled) {
                    NeckSetRotateSpeed(0.0);
                }
                break;
            default:
                break;
        }
    }

    private void ManageEncoderFailure() {
        if (Math.abs(neckEncoder.get() - backupNeckEncoder.get()) > 10) {
            // Encoder Fail/Drift, Automatically Kick into Manual Control for Safety
            manualControlEnabled = true;
        } else {
            manualControlEnabled = false;
        }
    }
}