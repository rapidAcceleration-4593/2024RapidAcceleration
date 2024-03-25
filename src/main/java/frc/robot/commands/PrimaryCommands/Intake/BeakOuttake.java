package frc.robot.commands.PrimaryCommands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PrimarySubsystem;

public class BeakOuttake extends Command {
    
    private final PrimarySubsystem primarySubsystem;

    public BeakOuttake(PrimarySubsystem primarySubsystem) {
        this.primarySubsystem = primarySubsystem;
        addRequirements(primarySubsystem);
    }

    @Override
    public void execute() {
        primarySubsystem.Outtake();
    }
}
