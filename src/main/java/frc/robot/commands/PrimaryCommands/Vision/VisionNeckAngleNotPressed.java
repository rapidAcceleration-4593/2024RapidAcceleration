package frc.robot.commands.PrimaryCommands.Vision;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.NeckSubsystem;

public class VisionNeckAngleNotPressed extends Command {
    
    private final NeckSubsystem neckSubsystem;

    public VisionNeckAngleNotPressed(NeckSubsystem neckSubsystem) {
        this.neckSubsystem = neckSubsystem;
        addRequirements(neckSubsystem);
    }

    @Override
    public void execute() {
        neckSubsystem.VisionNeckAngleNotPressed();
    }
}
