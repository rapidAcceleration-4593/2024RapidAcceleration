package frc.robot.commands.NeckRotationCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.NeckRotationSubsystem;

public class NeckFollower extends Command {
    
    private final NeckRotationSubsystem neckSubsystem;

    public NeckFollower(NeckRotationSubsystem neckSubsystem) {
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
