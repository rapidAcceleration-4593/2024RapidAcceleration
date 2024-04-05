package frc.robot.commands.BeakCommands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PrimarySubsystem;

public class BeakShooter extends Command {
    
    private final PrimarySubsystem primarySubsystem;

    public BeakShooter(PrimarySubsystem primarySubsystem) {
        this.primarySubsystem = primarySubsystem;
        addRequirements(primarySubsystem);
    }

    @Override
    public void execute() {
        primarySubsystem.Shooter();
    }
}
