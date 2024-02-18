package frc.robot.commands.NeckRotationCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.NeckRotationSubsystem;

public class NeckFollower extends Command {
    
    private final NeckRotationSubsystem neck;

    public NeckFollower(NeckRotationSubsystem neckPassedIn) {
        neck = neckPassedIn;
        addRequirements(neckPassedIn);
    }

    @Override
    public void execute() {
        neck.NeckFollower();
    }
}
