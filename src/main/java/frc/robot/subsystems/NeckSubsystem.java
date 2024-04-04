package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.*;

public class NeckSubsystem extends SubsystemBase {

    private final CANSparkMax leftGearbox1 = NeckConstants.leftGearbox1;
    private final CANSparkMax leftGearbox2 = NeckConstants.leftGearbox2;
    private final CANSparkMax rightGearbox1 = NeckConstants.rightGearbox1;
    private final CANSparkMax rightGearbox2 = NeckConstants.rightGearbox2;

    private final DigitalInput topLimitSwitch = NeckConstants.topLimitSwitch;
    private final DigitalInput bottomLimitSwitch = NeckConstants.bottomLimitSwitch;

    private final Encoder primaryNeckEncoder = NeckConstants.primaryNeckEncoder;
    private final DutyCycleEncoder secondaryNeckEncoder = NeckConstants.secondaryNeckEncoder;

    public enum NeckStates { INTAKE, SUBWOOFER, VISION, YEET, AMP, MANUAL_UP, MANUAL_DOWN, MANUAL_STOP }
    public NeckStates currentNeckState = NeckStates.INTAKE;

    public final PIDController neckRotateController = new PIDController(0.0, 0.0, 0.0);

    private boolean bottomLimitSwitchPressed = false;
    private boolean manualControlEnabled = false;

    public int neckGoalAngle = 0;

    public void IntakePosition() { if (!manualControlEnabled) { currentNeckState = NeckStates.INTAKE; }}
    public void SubwooferPosition() { if (!manualControlEnabled) { currentNeckState = NeckStates.SUBWOOFER; }}
    public void VisionNeckAnglePressed() { if (!manualControlEnabled && currentNeckState != NeckStates.INTAKE) { currentNeckState = NeckStates.VISION; }}
    public void VisionNeckAngleNotPressed() { if (!manualControlEnabled && currentNeckState == NeckStates.VISION) { currentNeckState = NeckStates.SUBWOOFER; }}
    public void YeetPosition() { if (!manualControlEnabled && currentNeckState != NeckStates.INTAKE) { currentNeckState = NeckStates.YEET; }}
    public void AmpPosition() { if (!manualControlEnabled && currentNeckState != NeckStates.INTAKE) { currentNeckState = NeckStates.AMP; }}

    public void NeckUp() { if (manualControlEnabled) { if (topLimitSwitch.get()) { currentNeckState = NeckStates.MANUAL_UP; } else { currentNeckState = NeckStates.MANUAL_STOP; }}}
    public void NeckDown() { if (manualControlEnabled) { if (bottomLimitSwitch.get()) { currentNeckState = NeckStates.MANUAL_DOWN; } else { currentNeckState = NeckStates.MANUAL_STOP; }}}
    public void NeckStop() { if (manualControlEnabled) { currentNeckState = NeckStates.MANUAL_STOP; }}

    public void EnableManualControl() { manualControlEnabled = true; }
    public void DisableManualControl() { if (!bottomLimitSwitch.get()) { currentNeckState = NeckStates.INTAKE; manualControlEnabled = false; }} // Math.abs(Math.abs(primaryNeckEncoder.get()) - Math.abs(secondaryNeckEncoder.get())) < 5 && 

    public void periodic() {
        ManageNeckStates();
        ManageEncoderFailure();

        if (!manualControlEnabled) {
            ManageLimitSwitches();
            UpdatePIDConstants();
        }

        System.out.println("<--------------->");
        System.out.println("Goal:" + neckGoalAngle);
        System.out.println("Primary Encoder Value: " + primaryNeckEncoder.get());
        System.out.println("Secondary Encoder Value: " + secondaryNeckEncoder.get());

        // System.out.println("Top Limit Switch: " + !topLimitSwitch.get());
        // System.out.println("Bottom Limit Switch: " + !bottomLimitSwitch.get());

        // System.out.println("Manual Control Enabled: " + manualControlEnabled);
        // System.out.println("Neck State: " + currentNeckState);
    }

    private void ManageNeckStates() {
        switch (currentNeckState) {
            case INTAKE:
                neckGoalAngle = 0;
                break;
            case SUBWOOFER:
                neckGoalAngle = 20;
                break;
            case VISION:
                boolean hasTargets = LimelightHelpers.getTV("");
                if (hasTargets && (LimelightHelpers.getFiducialID("") == 4 || LimelightHelpers.getFiducialID("") == 7)) {
                    neckGoalAngle = VisionSetAngle(Math.hypot(LimelightHelpers.getTargetPose3d_CameraSpace("").getZ(), Math.abs(LimelightHelpers.getTargetPose3d_CameraSpace("").getX())));
                }
                break;
            case YEET:
                neckGoalAngle = 100;
                break;
            case AMP:
                neckGoalAngle = 250;
                break;
            case MANUAL_UP:
                NeckSetRotateSpeed(0.16);
                break;
            case MANUAL_DOWN:
                NeckSetRotateSpeed(-0.07);
                break;
            case MANUAL_STOP:
                NeckSetRotateSpeed(0.0);
                break;
            default:
                break;
        }
    }

    private void UpdatePIDConstants() {
        if (!manualControlEnabled) {
            double p, i, d;
            double error = primaryNeckEncoder.get() - neckGoalAngle;

            if (Math.abs(error) <= 10 && currentNeckState != NeckStates.INTAKE && currentNeckState != NeckStates.AMP) {
                // Close Controller
                p = 0.008;
                i = 0.003;
                d = 0.0;
            } else if (error < 0 && primaryNeckEncoder.get() < 200 && currentNeckState != NeckStates.INTAKE) {
                // Up Controller
                p = 0.0015;
                i = 0.0075;
                d = 0.0002;
            } else if (error > 0 && primaryNeckEncoder.get() > 25 && currentNeckState != NeckStates.AMP) {
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
            NeckSetRotateSpeed(neckRotateController.calculate(primaryNeckEncoder.get(), neckGoalAngle));
        }
    }

    private void ManageLimitSwitches() {
        if (bottomLimitSwitch.get()) {
            if (currentNeckState == NeckStates.INTAKE && primaryNeckEncoder.get() <= 25 && primaryNeckEncoder.get() > 0) {
                NeckSetRotateSpeed(-0.07);
            }
            bottomLimitSwitchPressed = false;
        } else {
            if (!bottomLimitSwitchPressed) {
                primaryNeckEncoder.reset();
                secondaryNeckEncoder.reset();
                bottomLimitSwitchPressed = true;
            }
        }

        if (topLimitSwitch.get()) {
            if (currentNeckState == NeckStates.AMP && primaryNeckEncoder.get() >= 200 && primaryNeckEncoder.get() < 250) {
                NeckSetRotateSpeed(0.16);
            }
        }
    }

    private void ManageEncoderFailure() {
        // if (Math.abs(Math.abs(primaryNeckEncoder.get()) - Math.abs(secondaryNeckEncoder.get())) > 20) {
        //     manualControlEnabled = true;
        // }
    }

    private void NeckSetRotateSpeed(double speed) {
        leftGearbox1.set(speed);
        leftGearbox2.follow(leftGearbox1, false);
        rightGearbox1.follow(leftGearbox1, true);
        rightGearbox2.follow(leftGearbox1, true);
    }

    private int VisionSetAngle(double distance) {
        int angle = 0;
        if (distance > 3.3 && distance < 15) {
            angle = (int) (-349.351*Math.pow(distance, -0.854219) + 140.0);
        }
        return angle;
    }
}