package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.BeakConstants;
import frc.robot.Constants.NeckRotationConstants;

public class BeakSubsystem extends SubsystemBase {
    
    // Define Empty CANSparkMax Motor Controller Objects
    private final CANSparkMax intakeMotor;
    private final CANSparkMax shooterTopMotor;
    private final CANSparkMax shooterBottomMotor;

    // Define Empty Intake Limit Switch & Arm Angle Encoder
    private final DigitalInput intakeLimitSwitch;
    private final Encoder neckEncoder;

    // Define Timer for Delay Between Shooter and Intake Spinup
    private Timer shooterTimer;
    private boolean shooterTimerStarted;

    private boolean intakeIntaked = false;

    public BeakSubsystem() {
        // Define Empty Objects to IDs specified in the Constants.java File
        intakeMotor = BeakConstants.intakeMotor;
        shooterTopMotor = BeakConstants.shooterTopMotor;
        shooterBottomMotor = BeakConstants.shooterBottomMotor;

        intakeLimitSwitch = BeakConstants.intakeLimitSwitch;
        neckEncoder = NeckRotationConstants.neckEncoder;

        shooterTimer = new Timer();
        shooterTimerStarted = false;
    }

    public void BeakIntake() {
        // If there isn't an intake limit switch, then set motor speed when command is called
        // If the intake limit switch is pressed, then stop the intake motor, regardless of if the command is called or not
        intakeMotor.set(-1.0);
    }
    
    public void BeakOuttake() {
        // If the outtake command is called, spin the intake motor backward to shoot the note out if stuck
        intakeMotor.set(1.0);
    }

    public void BeakIntakeStop() {
        // When the intake and outtake commands are not called, stop the motor while false
        intakeMotor.set(0.0);
    }

    public void BeakShooter() {
        // If the arm angle is high enough for amp position, spin the motor down to 25%. If not, spin normally at full speed
        if (neckEncoder.get() >= 200) {
            shooterTopMotor.set(-0.25);
            shooterBottomMotor.set(-0.25);
        } else {
            shooterTopMotor.set(-1.0);
            shooterBottomMotor.set(-1.0);
        }

        // If the shooter timer has not started, then start the timer and set the variable to true
        if (!shooterTimerStarted) {
            shooterTimer.start();
            shooterTimerStarted = true;
        }

        // Once the shooter timer goes over 0.8 seconds, spin the intake to push the note into the shooter
        if (shooterTimer.get() >= 0.8) {
            intakeMotor.set(-1.0);
        }
    }
    
    public void BeakShooterStop() {
        // While the shooter command is false, call this method is stop both top and bottom motors, stop the intake, and stop and reset the shooter timer to run again
        shooterTopMotor.set(0.0);
        shooterBottomMotor.set(0.0);
        intakeMotor.set(0.0);
        shooterTimer.stop();
        shooterTimer.reset();
        shooterTimerStarted = false;
    }

    public void ShooterAuto() {
        // This command is only to be ran in auto, as this is an initialize command since auto doesn't accept execute commands
        shooterTopMotor.set(-1.0);
        shooterBottomMotor.set(-1.0);
    }

    public void IntakeAuto() {
        // This command is only to be ran in auto, as this is an initialize command since auto doesn't accept execute commands (meaning limit switch doesn't work)
        intakeMotor.set(-1.0);
    }

    // The initialize() method marks the command start, and is called exactly once per time a command is scheduled
    // The execute() method is called repeatedly while the command is scheduled

    public void periodic() {
        if (!intakeLimitSwitch.get() && intakeIntaked == false) {
            BeakIntakeStop();
            intakeIntaked = true;
        } else if (intakeLimitSwitch.get()) {
            intakeIntaked = false;
        }
    }
}
