package frc.robot.commands.NeckCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.NeckSubsystem;

public class NeckFollower extends Command {
    
    private final NeckSubsystem neckSubsystem;

    public NeckFollower(NeckSubsystem neckSubsystem) {
        this.neckSubsystem = neckSubsystem;
        addRequirements(neckSubsystem);
    }

    @Override
    public void execute() {
        neckSubsystem.NeckFollower();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
