package frc.robot.commands.NeckRotationCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.NeckRotationSubsystem;

public class VisionNeckAngle extends Command {
    
    private final NeckRotationSubsystem neckSubsystem;

    public VisionNeckAngle(NeckRotationSubsystem neckSubsystem) {
        this.neckSubsystem = neckSubsystem;
        addRequirements(neckSubsystem);
    }

    @Override
    public void initialize() {
        neckSubsystem.VisionNeckAngle();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
