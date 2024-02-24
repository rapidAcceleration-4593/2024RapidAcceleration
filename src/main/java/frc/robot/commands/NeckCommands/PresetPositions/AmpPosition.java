package frc.robot.commands.NeckCommands.PresetPositions;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.NeckSubsystem;

public class AmpPosition extends Command {
    
    private final NeckSubsystem neckSubsystem;

    public AmpPosition(NeckSubsystem neckSubsystem) {
        this.neckSubsystem = neckSubsystem;
        addRequirements(neckSubsystem);
    }

    @Override
    public void initialize() {
        neckSubsystem.AmpPosition();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
