package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.BeakSubsystem;

public class StopAllAuto extends Command {

    private final BeakSubsystem beakSubsystem;

    public StopAllAuto(BeakSubsystem beakSubsystem) {
        this.beakSubsystem = beakSubsystem;
        addRequirements(beakSubsystem);
    }
    
    @Override
    public void initialize() {
        beakSubsystem.StopAllAuto();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
