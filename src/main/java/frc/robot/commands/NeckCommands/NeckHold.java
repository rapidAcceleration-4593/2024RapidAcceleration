package frc.robot.commands.NeckCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.NeckSubsystem;

public class NeckHold extends Command {
    
    private final NeckSubsystem neck;

    public NeckHold(NeckSubsystem neckPassedIn) {
        neck = neckPassedIn;
        addRequirements(neckPassedIn);
    }

    @Override
    public void execute() {
        neck.NeckHold();
    }
}
