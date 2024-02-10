package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class ArmOuttake extends Command {
    
    private final ArmSubsystem arm;

    public ArmOuttake(ArmSubsystem armPassedIn) {
        arm = armPassedIn;
        addRequirements(armPassedIn);
    }

    public void execute() {
        arm.ArmOuttake();
    }
}
