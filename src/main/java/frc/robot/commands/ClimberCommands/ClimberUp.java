package frc.robot.commands.ClimberCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberUp extends Command {

    private final ClimberSubsystem climberSubsystem;

    public ClimberUp(ClimberSubsystem climberSubsystem) {
        this.climberSubsystem = climberSubsystem;
        addRequirements(climberSubsystem);
    }

    @Override
    public void initialize() {
        climberSubsystem.ClimberUp();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
