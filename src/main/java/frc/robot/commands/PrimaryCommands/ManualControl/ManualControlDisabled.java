package frc.robot.commands.PrimaryCommands.ManualControl;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PrimarySubsystem;

public class ManualControlDisabled extends Command {
    
    private final PrimarySubsystem primarySubsystem;

    public ManualControlDisabled(PrimarySubsystem primarySubsystem) {
        this.primarySubsystem = primarySubsystem;
        addRequirements(primarySubsystem);
    }

    @Override
    public void initialize() {
        primarySubsystem.DisableManualControl();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
