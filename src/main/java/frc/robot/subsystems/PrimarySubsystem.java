package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.*;

public class PrimarySubsystem extends SubsystemBase {

    private final CANSparkMax leftGearbox1 = NeckRotationConstants.leftGearbox1;
    private final CANSparkMax leftGearbox2 = NeckRotationConstants.leftGearbox2;
    private final CANSparkMax rightGearbox1 = NeckRotationConstants.rightGearbox1;
    private final CANSparkMax rightGearbox2 = NeckRotationConstants.rightGearbox2;

    private final CANSparkMax bumperIntakeMotor = BeakConstants.bumperIntakeMotor;
    private final CANSparkMax beakIntakeMotor = BeakConstants.beakIntakeMotor;
    private final CANSparkMax shooterTopMotor = BeakConstants.shooterTopMotor;
    private final CANSparkMax shooterBottomMotor = BeakConstants.shooterBottomMotor;

    private final DigitalInput intakeLimitSwitch = BeakConstants.intakeLimitSwitch;
    private final DigitalInput topLimitSwitch = NeckRotationConstants.topLimitSwitch;
    private final DigitalInput bottomLimitSwitch = NeckRotationConstants.bottomLimitSwitch;

    private final Encoder neckEncoder = NeckRotationConstants.neckEncoder;
    private final Encoder backupNeckEncoder = NeckRotationConstants.backupNeckEncoder;

    private final Relay siccLEDS = LEDConstants.SiccLEDs;

    private enum NeckStates { INTAKE, VISION, YEET, AMP, MANUAL_UP, MANUAL_DOWN, MANUAL_STOP }
    private NeckStates currentNeckState = NeckStates.INTAKE;

    public final PIDController neckRotateController = new PIDController(0.0, 0.0, 0.0);

    private boolean bottomLimitSwitchState = false;
    private boolean manualControlEnabled = false;
    private boolean visionButtonPressed = false;
    private boolean shooterTimerStarted = false;
    private boolean intakeIntaked = false;

    private Timer shooterTimer = new Timer();

    private int neckGoalAngle = 0;

    public void IntakePosition() { if (!manualControlEnabled) { currentNeckState = NeckStates.INTAKE; }}
    public void VisionNeckAngle() { if (!manualControlEnabled) { currentNeckState = NeckStates.VISION; visionButtonPressed = true; }}
    public void YeetPosition() { if (!manualControlEnabled) { currentNeckState = NeckStates.YEET; }}
    public void AmpPosition() { if (!manualControlEnabled) { currentNeckState = NeckStates.AMP; }}

    public void NeckUp() { if (manualControlEnabled) { if (topLimitSwitch.get()) { currentNeckState = NeckStates.MANUAL_UP; } else { currentNeckState = NeckStates.MANUAL_STOP; }}}
    public void NeckDown() { if (manualControlEnabled) { if (bottomLimitSwitch.get()) { currentNeckState = NeckStates.MANUAL_DOWN; } else { currentNeckState = NeckStates.MANUAL_STOP; }}}
    public void NeckStop() { if (manualControlEnabled) { currentNeckState = NeckStates.MANUAL_STOP; }}

    public void EnableManualControl() { manualControlEnabled = true; }
    public void DisableManualControl() { if (Math.abs(Math.abs(neckEncoder.get()) - Math.abs(backupNeckEncoder.get())) < 5 && !bottomLimitSwitch.get()) { currentNeckState = NeckStates.INTAKE; manualControlEnabled = false; }}

    private void UpdatePIDConstants() {
        double p, i, d;

        double error = neckEncoder.get() - neckGoalAngle;

        if (Math.abs(error) < 20 && neckEncoder.get() > 25 && neckEncoder.get() < 200) {
            // Close Controller
            p = 0.008;
            i = 0.003;
            d = 0.0;
        } else if (error < 0 && neckEncoder.get() < 235 && neckGoalAngle > 0) {
            // Up Controller
            p = 0.0015;
            i = 0.0075;
            d = 0.0002;
        } else if (error > 0 && neckEncoder.get() > 25) {
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

    private int VisionSetAngle(double distance) {
        int angle = 0;
        if (distance > 3.3 && distance < 13) {
            angle = (int) (-349.351*Math.pow(distance, -0.854219) + 172.0);
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
        ManageEncoderFailure();
        ManageNeckStates();

        if (!manualControlEnabled) {
            UpdatePIDConstants();
            NeckSetRotateSpeed(neckRotateController.calculate(neckEncoder.get(), neckGoalAngle));

            if (!bottomLimitSwitch.get()) {
                if (!bottomLimitSwitchState) {
                    neckEncoder.reset();
                    backupNeckEncoder.reset();
                    bottomLimitSwitchState = true;
                }
            } else {
                bottomLimitSwitchState = false;
                if (neckEncoder.get() <= 25 && neckEncoder.get() > 0 && neckGoalAngle == 0) {
                    NeckSetRotateSpeed(-0.07);
                }
            }

            if (topLimitSwitch.get() && neckGoalAngle == 200) {
                NeckSetRotateSpeed(0.15);
            } else if (!topLimitSwitch.get() && neckGoalAngle == 200) {
                NeckSetRotateSpeed(0.0);
            }
        }

        if (!intakeLimitSwitch.get() && !intakeIntaked) {
            IntakeStop();
            intakeIntaked = true; 
        } else if (!intakeLimitSwitch.get()) {
            siccLEDS.set(Relay.Value.kOn);
        } else if (intakeLimitSwitch.get()) {
            intakeIntaked = false;
            siccLEDS.set(Relay.Value.kOff);
        }

        System.out.println("<--------------->");
        System.out.println("Goal:" + neckGoalAngle);
        System.out.println("Encoder Value: " + neckEncoder.get());
        System.out.println("Backup Encoder Value: " + backupNeckEncoder.get());

        // System.out.println("Top Limit Switch: " + topLimitSwitch.get());
        // System.out.println("Bottom Limit Switch: " + bottomLimitSwitch.get());
        // System.out.println("Intake Limit Switch: " + intakeLimitSwitch.get());

        // System.out.println("Manual Control Enabled: " + manualControlEnabled);
        // System.out.println("Neck State: " + currentNeckState);
    }

    private void ManageNeckStates() {
        switch (currentNeckState) {
            case INTAKE:
                neckGoalAngle = 0;
                break;
            case VISION:
                if (visionButtonPressed) {
                    boolean hasTargets = LimelightHelpers.getTV("");
                    if (hasTargets && (LimelightHelpers.getFiducialID("") == 4 || LimelightHelpers.getFiducialID("") == 7)) {
                        neckGoalAngle = VisionSetAngle(LimelightHelpers.getTargetPose3d_CameraSpace("").getZ());
                        visionButtonPressed = false;
                    }
                }
                break;
            case YEET:
                neckGoalAngle = 100;
                break;
            case AMP:
                neckGoalAngle = 200;
                break;
            case MANUAL_UP:
                NeckSetRotateSpeed(0.16);
                break;
            case MANUAL_DOWN:
                NeckSetRotateSpeed(-0.06);
                break;
            case MANUAL_STOP:
                NeckSetRotateSpeed(0.0);
                break;
            default:
                break;
        }
    }

    private void ManageEncoderFailure() {
        int encoderDifference = Math.abs(neckEncoder.get()) - Math.abs(backupNeckEncoder.get());
        if (Math.abs(encoderDifference) > 20) {
            manualControlEnabled = true;
        }
    }

    public void Intake() {
        if (!bottomLimitSwitch.get() && intakeLimitSwitch.get() && currentNeckState == NeckStates.INTAKE) {
            bumperIntakeMotor.set(-1.0);
            beakIntakeMotor.set(-1.0);
        }
    }
    
    public void Outtake() {
        if (!bottomLimitSwitch.get() && currentNeckState == NeckStates.INTAKE) {
            bumperIntakeMotor.set(1.0);
            beakIntakeMotor.set(1.0);
        }
    }

    public void IntakeStop() {
        bumperIntakeMotor.set(0.0);
        beakIntakeMotor.set(0.0);
    }

    public void Shooter() {
        if (neckGoalAngle == 200) {
            shooterTopMotor.set(-0.25);
            shooterBottomMotor.set(-0.25);
        } else {
            shooterTopMotor.set(-1.0);
            shooterBottomMotor.set(-1.0);
        }

        if (!shooterTimerStarted) {
            shooterTimer.start();
            shooterTimerStarted = true;
        }

        if (shooterTimer.get() >= 0.8) {
            beakIntakeMotor.set(-1.0);
        }
    }
    
    public void ShooterStop() {
        shooterTopMotor.set(0.0);
        shooterBottomMotor.set(0.0);
        beakIntakeMotor.set(0.0);
        shooterTimer.stop();
        shooterTimer.reset();
        shooterTimerStarted = false;
    }

    public void ShooterAuto() {
        shooterTopMotor.set(-1.0);
        shooterBottomMotor.set(-1.0);
    }

    public void IntakeAuto() {
        bumperIntakeMotor.set(-1.0);
        beakIntakeMotor.set(-1.0);
    }

    public void StopAllAuto() {
        bumperIntakeMotor.set(0.0);
        beakIntakeMotor.set(0.0);
        shooterTopMotor.set(0.0);
        shooterBottomMotor.set(0.0);
    }
}