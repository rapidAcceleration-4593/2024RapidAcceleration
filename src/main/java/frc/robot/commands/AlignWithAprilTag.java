package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Vision;

public class AlignWithAprilTag extends Command {
    private final Vision m_vision;

    public AlignWithAprilTag(Vision visionPassedIn) {
        m_vision = visionPassedIn;
        addRequirements(visionPassedIn);
    }

    @Override
    public void execute() {
        m_vision.alignWithAprilTag();
    }
}
