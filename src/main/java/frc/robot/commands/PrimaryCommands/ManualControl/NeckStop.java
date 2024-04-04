package frc.robot.commands.PrimaryCommands.ManualControl;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.NeckSubsystem;

public class NeckStop extends Command {
    
    private final NeckSubsystem neckSubsystem;

    public NeckStop(NeckSubsystem neckSubsystem) {
        this.neckSubsystem = neckSubsystem;
        addRequirements(neckSubsystem);
    }

    @Override
    public void execute() {
        neckSubsystem.NeckStop();
    }
}
