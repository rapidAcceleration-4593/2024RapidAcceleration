package frc.robot.commands.NeckRotationCommands.PresetPositions;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.NeckRotationSubsystem;

public class IntakePosition extends Command {

    private final NeckRotationSubsystem neckSubsystem;

    public IntakePosition(NeckRotationSubsystem neckSubsystem) {
        this.neckSubsystem = neckSubsystem;
        addRequirements(neckSubsystem);
    }
    
    @Override
    public void initialize() {
        neckSubsystem.IntakePosition();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
