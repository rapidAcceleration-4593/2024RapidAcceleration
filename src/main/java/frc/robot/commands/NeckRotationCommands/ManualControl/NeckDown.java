package frc.robot.commands.NeckRotationCommands.ManualControl;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.NeckRotationSubsystem;

public class NeckDown extends Command {
    
    private final NeckRotationSubsystem neck;

    public NeckDown(NeckRotationSubsystem neckPassedIn) {
        neck = neckPassedIn;
        addRequirements(neckPassedIn);
    }

    public void execute() {
        neck.NeckDown();
    }
}
