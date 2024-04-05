package frc.robot.commands.BeakCommands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PrimarySubsystem;

public class BeakShooterStop extends Command {
    
    private final PrimarySubsystem primarySubsystem;

    public BeakShooterStop(PrimarySubsystem primarySubsystem) {
        this.primarySubsystem = primarySubsystem;
        addRequirements(primarySubsystem);
    }
    
    @Override
    public void execute() {
        primarySubsystem.ShooterStop();
    }
}
