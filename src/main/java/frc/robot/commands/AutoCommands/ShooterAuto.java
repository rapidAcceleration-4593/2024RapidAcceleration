package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PrimarySubsystem;

public class ShooterAuto extends Command {
    
    private final PrimarySubsystem primarySubsystem;

    public ShooterAuto(PrimarySubsystem primarySubsystem) {
        this.primarySubsystem = primarySubsystem;
        addRequirements(primarySubsystem);
    }

    @Override
    public void initialize() {
        primarySubsystem.ShooterAuto();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
