package frc.robot.commands.NeckRotationCommands.PresetPositions;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.NeckRotationSubsystem;

public class IntakePosition extends Command {

    private final NeckRotationSubsystem neck;

    public IntakePosition(NeckRotationSubsystem neckPassedIn) {
        neck = neckPassedIn;
        addRequirements(neckPassedIn);
    }
    
    public void execute() {
        neck.IntakePosition();
    }
}
