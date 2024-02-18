package frc.robot.commands.NeckCommands.PresetPositions;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.NeckSubsystem;

public class IntakePosition extends Command {

    private final NeckSubsystem neck;

    public IntakePosition(NeckSubsystem neckPassedIn) {
        neck = neckPassedIn;
        addRequirements(neckPassedIn);
    }
    
    public void execute() {
        neck.IntakePosition();
    }
}
