package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PrimarySubsystem;

public class IntakeAuto extends Command {
    
    private final PrimarySubsystem primarySubsystem;

    public IntakeAuto(PrimarySubsystem primarySubsystem) {
        this.primarySubsystem = primarySubsystem;
        addRequirements(primarySubsystem);
    }

    @Override
    public void initialize() {
        primarySubsystem.IntakeAuto();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
