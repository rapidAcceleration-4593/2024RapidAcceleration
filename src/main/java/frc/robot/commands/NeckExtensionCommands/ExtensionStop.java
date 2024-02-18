package frc.robot.commands.NeckExtensionCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.NeckExtensionSubsystem;

public class ExtensionStop extends Command {
    
    private final NeckExtensionSubsystem neck;

    public ExtensionStop(NeckExtensionSubsystem neckPassedIn) {
        neck = neckPassedIn;
        addRequirements(neckPassedIn);
    }

    public void execute() {
        neck.ExtensionStop();
    }
}
