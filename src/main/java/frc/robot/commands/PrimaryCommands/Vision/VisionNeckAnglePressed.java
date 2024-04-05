package frc.robot.commands.PrimaryCommands.Vision;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PrimarySubsystem;

public class VisionNeckAnglePressed extends Command {
    
    private final PrimarySubsystem primarySubsystem;

    public VisionNeckAnglePressed(PrimarySubsystem primarySubsystem) {
        this.primarySubsystem = primarySubsystem;
        addRequirements(primarySubsystem);
    }

    @Override
    public void execute() {
        primarySubsystem.VisionNeckAnglePressed();
    }
}
