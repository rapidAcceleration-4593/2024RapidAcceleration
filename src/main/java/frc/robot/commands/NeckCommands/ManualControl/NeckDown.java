package frc.robot.commands.NeckCommands.ManualControl;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.NeckSubsystem;

public class NeckDown extends Command {
    
    private final NeckSubsystem neck;

    public NeckDown(NeckSubsystem neckPassedIn) {
        neck = neckPassedIn;
        addRequirements(neckPassedIn);
    }

    public void execute() {
        neck.NeckDown();
    }
}
