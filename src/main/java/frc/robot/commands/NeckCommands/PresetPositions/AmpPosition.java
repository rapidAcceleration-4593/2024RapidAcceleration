package frc.robot.commands.NeckCommands.PresetPositions;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.NeckSubsystem;

public class AmpPosition extends Command {
    
    private final NeckSubsystem neck;

    public AmpPosition(NeckSubsystem neckPassedIn) {
        neck = neckPassedIn;
        addRequirements(neckPassedIn);
    }

    public void execute() {
        neck.AmpPosition();
    }
}
