package frc.robot.commands.ArmCommands.PresetArmPositions;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class ArmIntakePosition extends Command {
    
    private final ArmSubsystem arm;

    public ArmIntakePosition(ArmSubsystem armPassedIn) {
        arm = armPassedIn;
        addRequirements(armPassedIn);
    }

    public void execute() {
        arm.ArmIntakePosition();
    }
}
