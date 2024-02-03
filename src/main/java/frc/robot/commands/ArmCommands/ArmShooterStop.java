package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class ArmShooterStop extends Command {
    
    private final ArmSubsystem arm;

    public ArmShooterStop(ArmSubsystem armPassedIn) {
        arm = armPassedIn;
        addRequirements(armPassedIn);
    }
    
    public void execute() {
        arm.ArmShooterStop();
    }
}
