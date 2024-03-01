package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.BeakSubsystem;

public class ShooterStopAuto extends Command {
    
    private final BeakSubsystem beakSubsystem;

    public ShooterStopAuto(BeakSubsystem beakSubsystem) {
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
