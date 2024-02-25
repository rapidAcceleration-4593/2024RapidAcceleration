package frc.robot.commands.NeckCommands.PresetPositions;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.NeckSubsystem;

public class DrivingPosition extends Command {

    private final NeckSubsystem neckSubsystem;

    public DrivingPosition(NeckSubsystem neckSubsystem) {
        this.neckSubsystem = neckSubsystem;
        addRequirements(neckSubsystem);
    }

    @Override
    public void initialize() {
        neckSubsystem.DrivingPosition();
    }
    
    @Override
    public boolean isFinished() {
        return true;
    }
}
