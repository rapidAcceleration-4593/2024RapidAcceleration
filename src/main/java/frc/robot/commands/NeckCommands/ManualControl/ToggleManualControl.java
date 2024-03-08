package frc.robot.commands.NeckCommands.ManualControl;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.NeckSubsystem;

public class ToggleManualControl extends Command {
    
    private final NeckSubsystem neckSubsystem;

    public ToggleManualControl(NeckSubsystem neckSubsystem) {
        this.neckSubsystem = neckSubsystem;
        addRequirements(neckSubsystem);
    }

    @Override
    public void initialize() {
        neckSubsystem.ToggleManualControl();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
