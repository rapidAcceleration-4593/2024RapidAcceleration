package frc.robot.commands.ArmCommands.ManualControl;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class ArmRotateDown extends Command {
    
    private final ArmSubsystem arm;

    public ArmRotateDown(ArmSubsystem armPassedIn) {
        arm = armPassedIn;
        addRequirements(armPassedIn);
    }

    @Override
    public void execute() {
        arm.ArmRotateDown();
    }
}
