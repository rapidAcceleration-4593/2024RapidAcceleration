package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.BeakSubsystem;

public class ShooterAuto extends Command {
    
    private final BeakSubsystem beakSubsystem;

    public ShooterAuto(BeakSubsystem beakSubsystem) {
        this.beakSubsystem = beakSubsystem;
        addRequirements(beakSubsystem);
    }

    @Override
    public void initialize() {
        beakSubsystem.ShooterAuto();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
