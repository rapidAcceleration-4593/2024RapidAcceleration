package frc.robot.commands.ClimberCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberDown extends Command {

    private final ClimberSubsystem climber;

    public ClimberDown(ClimberSubsystem climberPassedIn) {
        climber = climberPassedIn;
        addRequirements(climberPassedIn);
    }

    public void execute() {
        climber.ClimberDown();
    }
}
