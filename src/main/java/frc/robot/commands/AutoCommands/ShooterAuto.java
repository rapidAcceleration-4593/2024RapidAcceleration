package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.BeakSubsystem;

public class ShooterAuto extends Command {
    
    private final BeakSubsystem beckSubsystem;

    public ShooterAuto(BeakSubsystem beckSubsystem) {
        this.beckSubsystem = beckSubsystem;
        addRequirements(beckSubsystem);
    }

    @Override
    public void initialize() {
        beckSubsystem.ShooterAuto();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
