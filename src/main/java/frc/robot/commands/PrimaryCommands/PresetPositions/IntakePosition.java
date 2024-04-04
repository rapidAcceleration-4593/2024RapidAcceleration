package frc.robot.commands.PrimaryCommands.PresetPositions;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.NeckSubsystem;

public class IntakePosition extends Command {

    private final NeckSubsystem neckSubsystem;

    public IntakePosition(NeckSubsystem neckSubsystem) {
        this.neckSubsystem = neckSubsystem;
        addRequirements(neckSubsystem);
    }
    
    @Override
    public void initialize() {
        neckSubsystem.IntakePosition();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
