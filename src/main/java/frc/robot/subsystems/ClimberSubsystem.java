package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {
    
    private CANSparkMax climberMotor;

    public ClimberSubsystem() {
        climberMotor = new CANSparkMax(10, MotorType.kBrushless);
    }

    public void ClimberUp() {
        climberMotor.set(0.75);
    }

    public void ClimberDown() {
        climberMotor.set(-1.0);
    }

    public void ClimberStop() {
        climberMotor.set(0.0);
    }
}
