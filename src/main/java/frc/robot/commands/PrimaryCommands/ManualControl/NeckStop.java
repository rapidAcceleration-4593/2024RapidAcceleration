package frc.robot.commands.PrimaryCommands.ManualControl;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PrimarySubsystem;

public class NeckStop extends Command {
    
    private final PrimarySubsystem primarySubsystem;

    public NeckStop(PrimarySubsystem primarySubsystem) {
        this.primarySubsystem = primarySubsystem;
        addRequirements(primarySubsystem);
    }

    @Override
    public void execute() {
        primarySubsystem.NeckStop();
    }
}
