package frc.robot.commands.NeckExtensionCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.NeckExtensionSubsystem;

public class ExtensionIn extends Command {

    private final NeckExtensionSubsystem neckSubsystem;

    public ExtensionIn(NeckExtensionSubsystem neckSubsystem) {
        this.neckSubsystem = neckSubsystem;
        addRequirements(neckSubsystem);
    }

    public void execute() {
        neckSubsystem.ExtensionIn();
    }
    
}
