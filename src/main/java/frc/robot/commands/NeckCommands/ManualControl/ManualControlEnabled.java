package frc.robot.commands.NeckCommands.ManualControl;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.NeckSubsystem;

public class ManualControlEnabled extends Command {
    
    private final NeckSubsystem neckSubsystem;

    public ManualControlEnabled(NeckSubsystem neckSubsystem) {
        this.neckSubsystem = neckSubsystem;
        addRequirements(neckSubsystem);
    }

    @Override
    public void initialize() {
        neckSubsystem.EnableManualControl();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
