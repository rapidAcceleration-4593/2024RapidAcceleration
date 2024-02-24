package frc.robot.commands.NeckCommands.PresetPositions;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.NeckSubsystem;

public class SubwooferPosition extends Command {
    
    private final NeckSubsystem neckSubsystem;

    public SubwooferPosition(NeckSubsystem neckSubsystem) {
        this.neckSubsystem = neckSubsystem;
        addRequirements(neckSubsystem);
    }

    @Override
    public void initialize() {
        neckSubsystem.SubwooferPosition();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
