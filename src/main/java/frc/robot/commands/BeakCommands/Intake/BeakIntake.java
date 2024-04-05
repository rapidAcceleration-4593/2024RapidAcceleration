package frc.robot.commands.BeakCommands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PrimarySubsystem;

public class BeakIntake extends Command {
    
    private final PrimarySubsystem primarySubsystem;

    public BeakIntake(PrimarySubsystem primarySubsystem) {
        this.primarySubsystem = primarySubsystem;
        addRequirements(primarySubsystem);
    }

    @Override
    public void execute() {
        primarySubsystem.Intake();
    }
}
