package frc.robot.commands.PrimaryCommands.Vision;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.VisionSubsystem;

public class VisionSwerveAlign extends Command {

    private final VisionSubsystem visionSubsystem;

    public VisionSwerveAlign(VisionSubsystem visionSubsystem) {
        this.visionSubsystem = visionSubsystem;
        addRequirements(visionSubsystem);
    }

    @Override
    public void execute() {
        visionSubsystem.VisionSwerveAlign();
    }
}