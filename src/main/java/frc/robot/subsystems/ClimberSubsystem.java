package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
    
    private final CANSparkMax climberMotor;

    public ClimberSubsystem() {
        climberMotor = ClimberConstants.climberMotor;
    }

    public void ClimberUp() {
        climberMotor.set(1);
    }

    public void ClimberDown() {
        climberMotor.set(-1);
    }

    public void ClimberStop() {
        climberMotor.set(0.0);
    }
}
