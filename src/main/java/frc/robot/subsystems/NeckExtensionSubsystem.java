package frc.robot.subsystems;

// import com.revrobotics.CANSparkMax;
// import com.revrobotics.CANSparkLowLevel.MotorType;
// import edu.wpi.first.wpilibj.DigitalInput;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class NeckExtensionSubsystem extends SubsystemBase{
    
    // private final CANSparkMax extensionMotor;
    // private final DigitalInput extensionLimitSwitch;

    public NeckExtensionSubsystem() {
        // extensionMotor = new CANSparkMax(30, MotorType.kBrushless);
        // extensionLimitSwitch = new DigitalInput(1);
    }

    public void ExtensionIn() {
        // extensionMotor.set(-1.0);
    }

    public void ExtensionOut() {
        // extensionMotor.set(1.0);
    }

    public void ExtensionStop() {
        // extensionMotor.set(0.0);
    }
}
