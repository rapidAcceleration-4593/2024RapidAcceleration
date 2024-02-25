package frc.robot.commands.ExtensionCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.NeckSubsystem;

public class ExtendStop extends Command {
    
    private final NeckSubsystem neckSubsystem;

    public ExtendStop(NeckSubsystem neckSubsystem) {
        this.neckSubsystem = neckSubsystem;
        addRequirements(neckSubsystem);
    }

    @Override
    public void execute() {
        neckSubsystem.ExtendStop();
    }
}
