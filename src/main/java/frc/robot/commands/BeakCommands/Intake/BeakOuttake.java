package frc.robot.commands.BeakCommands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PrimarySubsystem;

public class BeakOuttake extends Command {
    
    private final PrimarySubsystem beakSubsystem;

    public BeakOuttake(PrimarySubsystem beakSubsystem) {
        this.beakSubsystem = beakSubsystem;
        addRequirements(beakSubsystem);
    }

    @Override
    public void execute() {
        beakSubsystem.Outtake();
    }
}
