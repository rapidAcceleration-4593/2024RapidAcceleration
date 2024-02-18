package frc.robot.commands.NeckExtensionCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.NeckExtensionSubsystem;

public class ExtensionOut extends Command {
    
    private final NeckExtensionSubsystem neck;

    public ExtensionOut(NeckExtensionSubsystem neckPassedIn) {
        neck = neckPassedIn;
        addRequirements(neckPassedIn);
    }

    public void execute() {
        neck.ExtensionOut();
    }
}
