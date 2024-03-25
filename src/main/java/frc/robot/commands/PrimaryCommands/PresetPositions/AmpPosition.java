package frc.robot.commands.PrimaryCommands.PresetPositions;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PrimarySubsystem;

public class AmpPosition extends Command {
    
    private final PrimarySubsystem primarySubsystem;

    public AmpPosition(PrimarySubsystem primarySubsystem) {
        this.primarySubsystem = primarySubsystem;
        addRequirements(primarySubsystem);
    }

    @Override
    public void initialize() {
        primarySubsystem.AmpPosition();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
