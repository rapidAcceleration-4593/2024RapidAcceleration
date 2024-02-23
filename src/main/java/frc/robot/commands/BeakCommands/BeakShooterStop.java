package frc.robot.commands.BeakCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.BeakSubsystem;

public class BeakShooterStop extends Command {
    
    private final BeakSubsystem beakSubsystem;

    public BeakShooterStop(BeakSubsystem beakSubsystem) {
        this.beakSubsystem = beakSubsystem;
        addRequirements(beakSubsystem);
    }
    
    @Override
    public void initialize() {
        beakSubsystem.BeakShooterStop();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
