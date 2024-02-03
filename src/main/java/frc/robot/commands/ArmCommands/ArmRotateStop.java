package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class ArmRotateStop extends Command {
    
    private final ArmSubsystem arm;

    public ArmRotateStop(ArmSubsystem armPassedIn) {
        arm = armPassedIn;
        addRequirements(armPassedIn);
    }

    @Override
    public void execute() {
        arm.ArmRotateStop();
    }
}
