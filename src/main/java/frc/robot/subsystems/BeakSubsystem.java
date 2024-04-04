package frc.robot.subsystems;

import frc.robot.subsystems.NeckSubsystem.NeckStates;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;

public class BeakSubsystem extends SubsystemBase {
    
    private final NeckSubsystem neckSubsystem = new NeckSubsystem();

    private final PWMSparkMax bumperIntakeMotor = BeakConstants.bumperIntakeMotor;
    private final PWMSparkMax beakIntakeMotor = BeakConstants.beakIntakeMotor;
    private final PWMSparkMax shooterTopMotor = BeakConstants.shooterTopMotor;
    private final PWMSparkMax shooterBottomMotor = BeakConstants.shooterBottomMotor;

    private final DigitalInput bottomLimitSwitch = NeckConstants.bottomLimitSwitch;
    private final DigitalInput intakeLimitSwitch = BeakConstants.intakeLimitSwitch;

    private final Relay siccLEDS = LEDConstants.SiccLEDs;

    private boolean shooterTimerStarted = false;
    private boolean intakeIntaked = false;
    private boolean subwooferPositionSet = false;
    private boolean intakeDownTimerStarted = true;

    private Timer shooterTimer = new Timer();
    private Timer intakeDownTimer = new Timer();

    public void Intake() {
        if (!bottomLimitSwitch.get() && !intakeLimitSwitch.get() && neckSubsystem.currentNeckState == NeckStates.INTAKE) {
            bumperIntakeMotor.set(1.0);
            beakIntakeMotor.set(-1.0);
        } else {
            IntakeStop();
        }
    }
    
    public void Outtake() {
        if (neckSubsystem.currentNeckState == NeckStates.INTAKE) {
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
        if (neckSubsystem.currentNeckState == NeckStates.AMP) {
            shooterTopMotor.set(0.25);
            shooterBottomMotor.set(0.25);
        } else {
            shooterTopMotor.set(1.0);
            shooterBottomMotor.set(1.0);
        }

        if (!shooterTimerStarted) {
            shooterTimer.start();
            shooterTimerStarted = true;
        } else if (shooterTimer.hasElapsed(0.8)) {
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

    public void periodic() {
        if (!neckSubsystem.manualControlEnabled) {
            if (intakeLimitSwitch.get()) {
                siccLEDS.set(Relay.Value.kOn);
                if (!intakeIntaked) {
                    IntakeStop();
                    intakeIntaked = true;
                }

                if (!bottomLimitSwitch.get() && !subwooferPositionSet && neckSubsystem.currentNeckState == NeckStates.INTAKE) {
                    neckSubsystem.SubwooferPosition();
                    subwooferPositionSet = true;
                }
            } else {
                siccLEDS.set(Relay.Value.kOff);
                intakeIntaked = false;

                if (bottomLimitSwitch.get() && subwooferPositionSet && neckSubsystem.currentNeckState != NeckStates.INTAKE) {
                    if (!intakeDownTimerStarted) {
                        intakeDownTimer.start();
                        intakeDownTimerStarted = true;
                    } else if (intakeDownTimer.hasElapsed(0.5)) {
                        neckSubsystem.IntakePosition();
                        intakeDownTimer.stop();
                        intakeDownTimer.reset();
                        intakeDownTimerStarted = false;
                        subwooferPositionSet = false;
                    }
                }
            }
        }
    }
}
