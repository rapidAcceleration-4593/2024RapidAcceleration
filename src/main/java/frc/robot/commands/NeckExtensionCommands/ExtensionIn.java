package frc.robot.commands.NeckExtensionCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.NeckExtensionSubsystem;

public class ExtensionIn extends Command {

    private final NeckExtensionSubsystem neck;

    public ExtensionIn(NeckExtensionSubsystem neckPassedIn) {
        neck = neckPassedIn;
        addRequirements(neckPassedIn);
    }

    public void execute() {
        neck.ExtensionIn();
    }
    
}
