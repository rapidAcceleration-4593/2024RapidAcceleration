package frc.robot.commands.PrimaryCommands.PresetPositions;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PrimarySubsystem;

public class IntakePosition extends Command {

    private final PrimarySubsystem primarySubsystem;

    public IntakePosition(PrimarySubsystem primarySubsystem) {
        this.primarySubsystem = primarySubsystem;
        addRequirements(primarySubsystem);
    }
    
    @Override
    public void initialize() {
        primarySubsystem.IntakePosition();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
