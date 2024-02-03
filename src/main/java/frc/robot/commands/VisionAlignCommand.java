package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.VisionSubsystem;

public class VisionAlignCommand extends Command {

    private final VisionSubsystem vision;

    public VisionAlignCommand(VisionSubsystem visionPassedIn) {
        vision = visionPassedIn;
        addRequirements(visionPassedIn);
    }

    @Override
    public void execute() {
        vision.alignAprilTag();
    }
}