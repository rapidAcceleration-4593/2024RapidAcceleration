package frc.robot.commands.ClimberCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberStop extends Command {

    private final ClimberSubsystem climberSubsystem;

    public ClimberStop(ClimberSubsystem climberSubsystem) {
        this.climberSubsystem = climberSubsystem;
        addRequirements(climberSubsystem);
    }

    @Override
    public void initialize() {
        climberSubsystem.ClimberStop();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
