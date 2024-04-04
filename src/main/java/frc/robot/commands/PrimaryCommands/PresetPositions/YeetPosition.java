package frc.robot.commands.PrimaryCommands.PresetPositions;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.NeckSubsystem;

public class YeetPosition extends Command {
    
    private final NeckSubsystem neckSubsystem;

    public YeetPosition(NeckSubsystem neckSubsystem) {
        this.neckSubsystem = neckSubsystem;
        addRequirements(neckSubsystem);
    }

    @Override
    public void initialize() {
        neckSubsystem.YeetPosition();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
