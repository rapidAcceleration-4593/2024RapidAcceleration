package frc.robot.commands.NeckRotationCommands.PresetPositions;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.NeckRotationSubsystem;

public class SubwooferPosition extends Command {
    
    private final NeckRotationSubsystem neckSubsystem;

    public SubwooferPosition(NeckRotationSubsystem neckSubsystem) {
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
