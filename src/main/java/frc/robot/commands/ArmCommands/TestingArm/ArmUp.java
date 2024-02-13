package frc.robot.commands.ArmCommands.TestingArm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class ArmUp extends Command {
    
    private final ArmSubsystem arm;

    public ArmUp(ArmSubsystem armPassedIn) {
        arm = armPassedIn;
        addRequirements(armPassedIn);
    }

    public void execute() {
        arm.ArmUp();
    }
}
