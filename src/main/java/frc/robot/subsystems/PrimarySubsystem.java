package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.*;

public class PrimarySubsystem extends SubsystemBase {

    private final CANSparkMax leftGearbox1 = NeckConstants.leftGearbox1;
    private final CANSparkMax leftGearbox2 = NeckConstants.leftGearbox2;
    private final CANSparkMax rightGearbox1 = NeckConstants.rightGearbox1;
    private final CANSparkMax rightGearbox2 = NeckConstants.rightGearbox2;

    private final PWMSparkMax bumperIntakeMotor = BeakConstants.bumperIntakeMotor;
    private final PWMSparkMax beakIntakeMotor = BeakConstants.beakIntakeMotor;
    private final PWMSparkMax shooterTopMotor = BeakConstants.shooterTopMotor;
    private final PWMSparkMax shooterBottomMotor = BeakConstants.shooterBottomMotor;

    private final DigitalInput intakeLimitSwitch = BeakConstants.intakeLimitSwitch;
    private final DigitalInput topLimitSwitch = NeckConstants.topLimitSwitch;
    private final DigitalInput bottomLimitSwitch = NeckConstants.bottomLimitSwitch;

    private final Encoder primaryNeckEncoder = NeckConstants.primaryNeckEncoder;
    private final DutyCycleEncoder secondaryNeckEncoder = NeckConstants.secondaryNeckEncoder;

    private enum NeckStates { INTAKE, SUBWOOFER, VISION, YEET, AMP, MANUAL_UP, MANUAL_DOWN, MANUAL_STOP }
    private NeckStates currentNeckState = NeckStates.INTAKE;

    private final PIDController neckRotateController = new PIDController(0.0, 0.0, 0.0);

    private boolean bottomLimitSwitchPressed = false;
    private boolean manualControlEnabled = false;
    private boolean shooterTimerStarted = false;
    private boolean intakeIntaked = false;
    private boolean subwooferPositionSet = false;
    private boolean intakeDownTimerStarted = true;

    private Timer shooterTimer = new Timer();
    private Timer intakeDownTimer = new Timer();

    private final Relay siccLEDS = LEDConstants.SiccLEDs;

    private int neckGoalAngle = 0;

    public void IntakePosition() { if (!manualControlEnabled) { currentNeckState = NeckStates.INTAKE; subwooferPositionSet = false; }}
    public void SubwooferPosition() { if (!manualControlEnabled) { currentNeckState = NeckStates.SUBWOOFER; subwooferPositionSet = true; }}
    public void VisionNeckAnglePressed() { if (!manualControlEnabled && currentNeckState != NeckStates.INTAKE) { currentNeckState = NeckStates.VISION; }}
    public void VisionNeckAngleNotPressed() { if (!manualControlEnabled && currentNeckState == NeckStates.VISION) { currentNeckState = NeckStates.SUBWOOFER; }}
    public void YeetPosition() { if (!manualControlEnabled && currentNeckState != NeckStates.INTAKE) { currentNeckState = NeckStates.YEET; }}
    public void AmpPosition() { if (!manualControlEnabled && currentNeckState != NeckStates.INTAKE) { currentNeckState = NeckStates.AMP; }}

    public void NeckUp() { if (manualControlEnabled) { if (topLimitSwitch.get()) { currentNeckState = NeckStates.MANUAL_UP; } else { currentNeckState = NeckStates.MANUAL_STOP; }}}
    public void NeckDown() { if (manualControlEnabled) { if (bottomLimitSwitch.get()) { currentNeckState = NeckStates.MANUAL_DOWN; } else { currentNeckState = NeckStates.MANUAL_STOP; }}}
    public void NeckStop() { if (manualControlEnabled) { currentNeckState = NeckStates.MANUAL_STOP; }}

    public void EnableManualControl() { manualControlEnabled = true; }
    public void DisableManualControl() { manualControlEnabled = false; }

    public void periodic() {
        ManageNeckStates();
        ManageEncoderFailure();
        UpdatePIDConstants();
        ManageLimitSwitches();

        System.out.println("<--------------->");
        System.out.println("Goal:" + neckGoalAngle);
        System.out.println("Primary Encoder Value: " + primaryNeckEncoder.get());
        // System.out.println("Secondary Encoder Value: " + secondaryNeckEncoder.get());

        // System.out.println("Top Limit Switch: " + !topLimitSwitch.get());
        // System.out.println("Bottom Limit Switch: " + !bottomLimitSwitch.get());

        // System.out.println("Manual Control Enabled: " + manualControlEnabled);
        System.out.println("Neck State: " + currentNeckState);


        if (!manualControlEnabled) {
            if (intakeLimitSwitch.get()) {
                siccLEDS.set(Relay.Value.kOn);
                intakeDownTimerStarted = false;
                if (!intakeIntaked) {
                    IntakeStop();
                    intakeIntaked = true;
                }

                if (!bottomLimitSwitch.get() && !subwooferPositionSet && currentNeckState == NeckStates.INTAKE) {
                    SubwooferPosition();
                    subwooferPositionSet = true;
                }
            } else {
                siccLEDS.set(Relay.Value.kOff);
                intakeIntaked = false;

                if (bottomLimitSwitch.get() && subwooferPositionSet && currentNeckState != NeckStates.INTAKE) {
                    if (!intakeDownTimerStarted) {
                        intakeDownTimer.start();
                        intakeDownTimerStarted = true;
                    }
                    
                    if (intakeDownTimer.hasElapsed(1.0)) {
                        IntakePosition();
                        intakeDownTimer.stop();
                        intakeDownTimer.reset();
                        intakeDownTimerStarted = false;
                        subwooferPositionSet = false;
                    }
                }
            }
        }
    }

    private void ManageNeckStates() {
        switch (currentNeckState) {
            case INTAKE:
                neckGoalAngle = 0;
                break;
            case SUBWOOFER:
                neckGoalAngle = 40;
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

            if (Math.abs(error) <= 15 && currentNeckState != NeckStates.INTAKE && currentNeckState != NeckStates.AMP) {
                // Close Controller
                p = 0.0155; // 0.008
                i = 0.002; // 0.003
                d = 0.0;
                neckRotateController.setPID(p, i, d);
                NeckSetRotateSpeed(neckRotateController.calculate(primaryNeckEncoder.get(), neckGoalAngle));
            } else if (error < 0 && primaryNeckEncoder.get() < 200 && currentNeckState != NeckStates.INTAKE) {
                // Up Controller
                p = 0.0015;
                i = 0.0075;
                d = 0.0002;
                neckRotateController.setPID(p, i, d);
                NeckSetRotateSpeed(neckRotateController.calculate(primaryNeckEncoder.get(), neckGoalAngle));
            } else if (error > 0 && primaryNeckEncoder.get() > 25 && currentNeckState != NeckStates.AMP) {
                // Down Controller
                p = 0.0016;
                i = 0.0;
                d = 0.0;
                neckRotateController.setPID(p, i, d);
                NeckSetRotateSpeed(neckRotateController.calculate(primaryNeckEncoder.get(), neckGoalAngle));
            } else {
                // Default Controller
                p = 0.0;
                i = 0.0;
                d = 0.0;
            }
        }
    }

    private void ManageLimitSwitches() {
        if (!manualControlEnabled) {
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
    }

    private void ManageEncoderFailure() {
        // if (!manualControlEnabled && Math.abs(Math.abs(primaryNeckEncoder.get()) - Math.abs(secondaryNeckEncoder.get())) > 20) {
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
            angle = (int) (-349.351*Math.pow(distance, -0.854219) + 156.0);
        }
        return angle;
    }




    public void Intake() {
        if (!bottomLimitSwitch.get() && !intakeLimitSwitch.get() && currentNeckState == NeckStates.INTAKE) {
            bumperIntakeMotor.set(1.0);
            beakIntakeMotor.set(-1.0);
        } else {
            IntakeStop();
        }
    }
    
    public void Outtake() {
        if (currentNeckState == NeckStates.INTAKE) {
            bumperIntakeMotor.set(-1.0);
            beakIntakeMotor.set(1.0);
        } else {
            IntakeStop();
        }
    }

    public void IntakeStop() {
        bumperIntakeMotor.set(0.0);
        beakIntakeMotor.set(0.0);
    }

    public void Shooter() {
        if (currentNeckState == NeckStates.AMP) {
            shooterTopMotor.set(0.25);
            shooterBottomMotor.set(0.25);
        } else {
            shooterTopMotor.set(1.0);
            shooterBottomMotor.set(1.0);
        }

        if (!shooterTimerStarted) {
            shooterTimer.start();
            shooterTimerStarted = true;
        } else if (shooterTimer.hasElapsed(1.5)) {
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
        shooterTopMotor.set(1.0);
        shooterBottomMotor.set(1.0);
    }

    public void IntakeAuto() {
        bumperIntakeMotor.set(1.0);
        beakIntakeMotor.set(-1.0);
    }

    public void StopAllAuto() {
        bumperIntakeMotor.set(0.0);
        beakIntakeMotor.set(0.0);
        shooterTopMotor.set(0.0);
        shooterBottomMotor.set(0.0);
    }
}