package frc.robot.commands.PrimaryCommands.PresetPositions;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PrimarySubsystem;

public class YeetPosition extends Command {
    
    private final PrimarySubsystem primarySubsystem;

    public YeetPosition(PrimarySubsystem primarySubsystem) {
        this.primarySubsystem = primarySubsystem;
        addRequirements(primarySubsystem);
    }

    @Override
    public void initialize() {
        primarySubsystem.YeetPosition();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
