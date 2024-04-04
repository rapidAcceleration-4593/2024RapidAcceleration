package frc.robot.commands.BeakCommands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.BeakSubsystem;

public class BeakOuttake extends Command {
    
    private final BeakSubsystem beakSubsystem;

    public BeakOuttake(BeakSubsystem beakSubsystem) {
        this.beakSubsystem = beakSubsystem;
        addRequirements(beakSubsystem);
    }

    @Override
    public void execute() {
        beakSubsystem.Outtake();
    }
}
