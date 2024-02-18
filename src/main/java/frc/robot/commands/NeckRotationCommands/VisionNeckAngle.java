package frc.robot.commands.NeckRotationCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.NeckRotationSubsystem;

public class VisionNeckAngle extends Command {
    
    private final NeckRotationSubsystem neck;

    public VisionNeckAngle(NeckRotationSubsystem neckPassedIn) {
        neck = neckPassedIn;
        addRequirements(neckPassedIn);
    }

    @Override
    public void execute() {
        neck.VisionNeckAngle();
    }
}
