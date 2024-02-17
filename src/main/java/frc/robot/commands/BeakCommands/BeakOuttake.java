package frc.robot.commands.BeakCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.BeakSubsystem;

public class BeakOuttake extends Command {
    
    private final BeakSubsystem beak;

    public BeakOuttake(BeakSubsystem beakPassedIn) {
        beak = beakPassedIn;
        addRequirements(beakPassedIn);
    }

    public void execute() {
        beak.BeakOuttake();
    }
}
