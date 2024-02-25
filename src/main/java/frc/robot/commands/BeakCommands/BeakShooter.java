package frc.robot.commands.BeakCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.BeakSubsystem;

public class BeakShooter extends Command {
    
    private final BeakSubsystem beakSubsystem;

    public BeakShooter(BeakSubsystem beakSubsystem) {
        this.beakSubsystem = beakSubsystem;
        addRequirements(beakSubsystem);
    }

    @Override
    public void execute() {
        beakSubsystem.BeakShooter();
    }
}
