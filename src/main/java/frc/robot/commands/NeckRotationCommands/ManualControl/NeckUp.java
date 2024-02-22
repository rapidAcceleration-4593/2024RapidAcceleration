package frc.robot.commands.NeckRotationCommands.ManualControl;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.NeckRotationSubsystem;

public class NeckUp extends Command {
    
    private final NeckRotationSubsystem neckSubsystem;

    public NeckUp(NeckRotationSubsystem neckSubsystem) {
        this.neckSubsystem = neckSubsystem;
        addRequirements(neckSubsystem);
    }

    public void execute() {
        neckSubsystem.NeckUp();
    }
}
