package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PrimarySubsystem;

public class StopAllAuto extends Command {

    private final PrimarySubsystem primarySubsystem;

    public StopAllAuto(PrimarySubsystem primarySubsystem) {
        this.primarySubsystem = primarySubsystem;
        addRequirements(primarySubsystem);
    }
    
    @Override
    public void initialize() {
        primarySubsystem.StopAllAuto();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
