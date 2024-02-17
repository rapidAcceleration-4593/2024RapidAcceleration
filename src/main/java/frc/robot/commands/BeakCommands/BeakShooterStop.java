package frc.robot.commands.BeakCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.BeakSubsystem;

public class BeakShooterStop extends Command {
    
    private final BeakSubsystem beak;

    public BeakShooterStop(BeakSubsystem beakPassedIn) {
        beak = beakPassedIn;
        addRequirements(beakPassedIn);
    }
    
    public void execute() {
        beak.BeakShooterStop();
    }
}
