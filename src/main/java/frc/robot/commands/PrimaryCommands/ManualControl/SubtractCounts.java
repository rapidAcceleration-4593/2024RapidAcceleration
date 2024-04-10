package frc.robot.commands.PrimaryCommands.ManualControl;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PrimarySubsystem;

public class SubtractCounts extends Command {
    
    private final PrimarySubsystem primarySubsystem;

    public SubtractCounts(PrimarySubsystem primarySubsystem) {
        this.primarySubsystem = primarySubsystem;
        addRequirements(primarySubsystem);
    }

    @Override
    public void initialize() {
        primarySubsystem.SubtractCounts();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
