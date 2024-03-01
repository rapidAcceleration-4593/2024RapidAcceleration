package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.BeakSubsystem;

public class IntakeAuto extends Command {
    
    private final BeakSubsystem beakSubsystem;

    public IntakeAuto(BeakSubsystem beakSubsystem) {
        this.beakSubsystem = beakSubsystem;
        addRequirements(beakSubsystem);
    }

    @Override
    public void initialize() {
        beakSubsystem.IntakeAuto();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
