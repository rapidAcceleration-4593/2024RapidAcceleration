package frc.robot.commands.PrimaryCommands.ManualControl;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.NeckSubsystem;

public class NeckUp extends Command {
    
    private final NeckSubsystem neckSubsystem;

    public NeckUp(NeckSubsystem neckSubsystem) {
        this.neckSubsystem = neckSubsystem;
        addRequirements(neckSubsystem);
    }

    @Override
    public void execute() {
        neckSubsystem.NeckUp();
    }
}
