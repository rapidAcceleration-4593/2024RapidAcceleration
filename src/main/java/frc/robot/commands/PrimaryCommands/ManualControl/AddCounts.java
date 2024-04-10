package frc.robot.commands.PrimaryCommands.ManualControl;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PrimarySubsystem;

public class AddCounts extends Command {
    
    private final PrimarySubsystem primarySubsystem;

    public AddCounts(PrimarySubsystem primarySubsystem) {
        this.primarySubsystem = primarySubsystem;
        addRequirements(primarySubsystem);
    }

    @Override
    public void initialize() {
        primarySubsystem.AddCounts();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
