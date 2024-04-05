package frc.robot.commands.PrimaryCommands.ManualControl;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PrimarySubsystem;

public class ManualControlEnabled extends Command {
    
    private final PrimarySubsystem primarySubsystem;

    public ManualControlEnabled(PrimarySubsystem primarySubsystem) {
        this.primarySubsystem = primarySubsystem;
        addRequirements(primarySubsystem);
    }

    @Override
    public void initialize() {
        primarySubsystem.EnableManualControl();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
