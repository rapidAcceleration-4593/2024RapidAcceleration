package frc.robot.commands.NeckCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.NeckSubsystem;

public class VisionNeckAngle extends Command {
    
    private final NeckSubsystem neck;

    public VisionNeckAngle(NeckSubsystem neckPassedIn) {
        neck = neckPassedIn;
        addRequirements(neckPassedIn);
    }

    @Override
    public void execute() {
        neck.VisionNeckAngle();
    }
}
