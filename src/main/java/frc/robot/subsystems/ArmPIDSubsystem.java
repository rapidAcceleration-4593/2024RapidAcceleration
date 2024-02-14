package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;

public class ArmPIDSubsystem extends PIDSubsystem {
    
    private final CANSparkMax leftGearbox1;
    private final CANSparkMax leftGearbox2;
    private final CANSparkMax rightGearbox1;
    private final CANSparkMax rightGearbox2;

    private final Encoder armEncoder;

    public ArmPIDSubsystem() {
        super(new PIDController(0.1, 0.0, 0.0)); // Adjust PID Constants

        leftGearbox1 = new CANSparkMax(19, MotorType.kBrushless);
        leftGearbox2 = new CANSparkMax(20, MotorType.kBrushless);
        rightGearbox1 = new CANSparkMax(8, MotorType.kBrushless);
        rightGearbox2 = new CANSparkMax(9, MotorType.kBrushless);

        armEncoder = new Encoder(0, 1); // Adjust channels accordingly

        getController().setTolerance(0.5); // Tolerance in degrees
        setSetpoint(0); // Initial setpoint
    }

    @Override
    public void useOutput(double output, double setpoint) {
        // Use the output to control the arm motors
        leftGearbox1.set(output);
        leftGearbox2.follow(leftGearbox1, false);
        rightGearbox1.follow(leftGearbox1, true);
        rightGearbox2.follow(leftGearbox1, true);
    }

    @Override
    public double getMeasurement() {
        return armEncoder.get();
    }
}
