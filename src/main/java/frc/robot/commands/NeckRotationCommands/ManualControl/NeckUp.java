package frc.robot.commands.NeckRotationCommands.ManualControl;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.NeckRotationSubsystem;

public class NeckUp extends Command {
    
    private final NeckRotationSubsystem neck;

    public NeckUp(NeckRotationSubsystem neckPassedIn) {
        neck = neckPassedIn;
        addRequirements(neckPassedIn);
    }

    public void execute() {
        neck.NeckUp();
    }
}
