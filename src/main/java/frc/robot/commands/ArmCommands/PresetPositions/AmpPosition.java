package frc.robot.commands.ArmCommands.PresetPositions;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class AmpPosition extends Command {
    
    private final ArmSubsystem arm;

    public AmpPosition(ArmSubsystem armPassedIn) {
        arm = armPassedIn;
        addRequirements(armPassedIn);
    }

    public void execute() {
        arm.AmpPosition();
    }
}
