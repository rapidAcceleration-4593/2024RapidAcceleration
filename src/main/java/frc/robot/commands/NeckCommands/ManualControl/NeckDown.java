package frc.robot.commands.NeckCommands.ManualControl;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.NeckSubsystem;

public class NeckDown extends Command {
    
    private final NeckSubsystem neckSubsystem;

    public NeckDown(NeckSubsystem neckSubsystem) {
        this.neckSubsystem = neckSubsystem;
        addRequirements(neckSubsystem);
    }

    public void execute() {
        neckSubsystem.NeckDown();
    }
}
